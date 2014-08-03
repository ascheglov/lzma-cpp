// C++ LZMA Decoder, core part
// Original code by Igor Pavlov (LZMA SDK 9.20)
// Ported to C++ by Anatoly Scheglov
// Placed in the public domain

#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <stdexcept>

namespace lzma
{
    typedef std::uint8_t Byte;

#if _MSC_VER <= 1800
#   define LZMA_NOEXCEPT throw()
#else
#   define LZMA_NOEXCEPT noexcept
#endif

    struct BadStream : std::exception
    {
        BadStream() {}
        virtual const char* what() const LZMA_NOEXCEPT override { return "invalid LZMA stream"; }
    };

    struct DictView
    {
        Byte* mem; ///< pointer to memory block
        std::size_t size; ///< total size
        std::size_t pos; ///< decoded size
    };

    /* There are two types of LZMA streams:
    0) Stream with end mark. That end mark adds about 6 bytes to compressed size.
    1) Stream without end mark. You must know exact uncompressed size to decompress such stream.
    */
    enum class FinishMode
    {
        Any,   ///< finish at any point
        End    ///< block must be finished at the end
    };

    /* ELzmaFinishMode has meaning only if the decoding reaches output limit !!!

    You must use LZMA_FINISH_END, when you know that current output buffer
    covers last bytes of block. In other cases you must use LZMA_FINISH_ANY.

    If LZMA decoder sees end marker before reaching output limit, it returns SZ_OK,
    and output value of destLen will be less than output buffer size limit.
    You can check status result also.

    You can use multiple checks to test data integrity after full decompression:
    1) Check Result and "status" variable.
    2) Check that output(destLen) = uncompressedSize, if you know real uncompressedSize.
    3) Check that output(srcLen) = compressedSize, if you know real compressedSize.
    You must use correct finish mode in that case. */

    enum class Status
    {
        NotSpecified,               ///< use main error code instead
        FinishedWithMark,           ///< stream was finished with end mark.
        NotFinished,                ///< stream was not finished
        NeedsMoreInput,             ///< you must provide more input bytes
        MaybeFinishedWithoutMark    ///< there is probability that stream was finished without end mark
    };

    /* ELzmaStatus is used only as output value for function call */

    /* ---------- LZMA Decoder state ---------- */

    /* LZMA_REQUIRED_INPUT_MAX = number of required input bytes for worst case.
    Num bits = log2((2^11 / 31) ^ 22) + 26 < 134 + 26 = 160; */

    namespace details
    {
        typedef std::uint32_t UInt32;
        typedef std::uint32_t Prob; // can be std::uint16_t

        struct Properties
        {
            unsigned lc, lp, pb;
            unsigned dicSize;
        };

        class DecoderCore
        {
        private:
            static const auto kNumTopBits = 24;
            static const auto kTopValue = 1u << kNumTopBits;

            static const auto kNumBitModelTotalBits = 11;
            static const auto kBitModelTotal = 1 << kNumBitModelTotalBits;
            static const auto kNumMoveBits = 5;

            static const auto kNumPosBitsMax = 4;
            static const auto kNumPosStatesMax = 1 << kNumPosBitsMax;

            static const auto kLenNumLowBits = 3;
            static const auto kLenNumLowSymbols = 1 << kLenNumLowBits;
            static const auto kLenNumMidBits = 3;
            static const auto kLenNumMidSymbols = 1 << kLenNumMidBits;
            static const auto kLenNumHighBits = 8;
            static const auto kLenNumHighSymbols = 1 << kLenNumHighBits;

            static const auto LenChoice = 0;
            static const auto LenChoice2 = LenChoice + 1;
            static const auto LenLow = LenChoice2 + 1;
            static const auto LenMid = LenLow + (kNumPosStatesMax << kLenNumLowBits);
            static const auto LenHigh = LenMid + (kNumPosStatesMax << kLenNumMidBits);
            static const auto kNumLenProbs = LenHigh + kLenNumHighSymbols;

            static const auto kNumStates = 12;
            static const auto kNumLitStates = 7;

            static const auto kStartPosModelIndex = 4;
            static const auto kEndPosModelIndex = 14;
            static const auto kNumFullDistances = 1 << (kEndPosModelIndex >> 1);

            static const auto kNumPosSlotBits = 6;
            static const auto kNumLenToPosStates = 4;

            static const auto kNumAlignBits = 4;
            static const auto kAlignTableSize = 1 << kNumAlignBits;

            static const auto kMatchMinLen = 2;
            static const auto kMatchSpecLenStart = kMatchMinLen + kLenNumLowSymbols + kLenNumMidSymbols + kLenNumHighSymbols;

            static const auto IsMatch = 0;
            static const auto IsRep = IsMatch + (kNumStates << kNumPosBitsMax);
            static const auto IsRepG0 = IsRep + kNumStates;
            static const auto IsRepG1 = IsRepG0 + kNumStates;
            static const auto IsRepG2 = IsRepG1 + kNumStates;
            static const auto IsRep0Long = IsRepG2 + kNumStates;
            static const auto PosSlot = IsRep0Long + (kNumStates << kNumPosBitsMax);
            static const auto SpecPos = PosSlot + (kNumLenToPosStates << kNumPosSlotBits);
            static const auto Align = SpecPos + kNumFullDistances - kEndPosModelIndex;
            static const auto LenCoder = Align + kAlignTableSize;
            static const auto RepLenCoder = LenCoder + kNumLenProbs;
            static const auto Literal = RepLenCoder + kNumLenProbs;

            static const auto LZMA_LIT_SIZE = 768;

        public:
            static const auto LZMA_REQUIRED_INPUT_MAX = 20u;

            static const auto RC_INIT_SIZE = 5u;

            DecoderCore() {}

            static std::size_t calcProbSize(unsigned lcPlusLp)
            {
                const auto LZMA_BASE_SIZE = 1846u;
                static_assert(Literal == LZMA_BASE_SIZE, "a bug detected");

                return LZMA_BASE_SIZE + (LZMA_LIT_SIZE << lcPlusLp);
            }

            void InitDicAndState(bool initDic, bool initState)
            {
                needFlush = true;
                remainLen = 0;
                tempBufSize = 0;

                if (initDic)
                {
                    processedPos = 0;
                    checkDicSize = 0;
                    needInitState = true;
                }

                if (initState)
                    needInitState = true;
            }

            /// Internal. (Used by LZMA2 decoder)
            void UpdateWithUncompressed(const void* src, std::size_t size) 
            {
                memcpy(m_dic.mem + m_dic.pos, src, size);
                m_dic.pos += size;

                if (this->checkDicSize == 0 && this->m_properties.dicSize - this->processedPos <= size)
                    this->checkDicSize = this->m_properties.dicSize;

                this->processedPos += size;
            }

            /** The decoding to internal dictionary buffer (CLzmaDec::dic).

            You must manually update CLzmaDec::dicPos, if it reaches CLzmaDec::dicBufSize !!!

            finishMode:
                It has meaning only if the decoding reaches output limit (dicLimit).
                    LZMA_FINISH_ANY - Decode just dicLimit bytes.
                    LZMA_FINISH_END - Stream must be finished after dicLimit.

            status:
                Status::FINISHED_WITH_MARK
                Status::NOT_FINISHED
                Status::NEEDS_MORE_INPUT
                Status::MAYBE_FINISHED_WITHOUT_MARK
            */
            void DecodeToDic(std::size_t dicLimit, const void* src, std::size_t& srcLen, FinishMode finishMode, Status& status)
            {
                auto srcBytes = static_cast<const Byte*>(src);
                auto inSize = srcLen;
                srcLen = 0;
                WriteRem(dicLimit);

                status = Status::NotSpecified;

                while (this->remainLen != kMatchSpecLenStart)
                {
                    if (this->needFlush)
                    {
                        for (; inSize > 0 && this->tempBufSize < RC_INIT_SIZE; srcLen++, inSize--)
                            this->tempBuf[this->tempBufSize++] = *srcBytes++;

                        if (this->tempBufSize < RC_INIT_SIZE)
                        {
                            status = Status::NeedsMoreInput;
                            return;
                        }

                        if (tempBuf[0] != 0)
                            throw BadStream();

                        InitRc(tempBuf);
                        tempBufSize = 0;
                    }

                    auto checkEndMarkNow = false;
                    if (m_dic.pos >= dicLimit)
                    {
                        if (this->remainLen == 0 && this->m_code == 0)
                        {
                            status = Status::MaybeFinishedWithoutMark;
                            return;
                        }
                        if (finishMode == FinishMode::Any)
                        {
                            status = Status::NotFinished;
                            return;
                        }
                        if (this->remainLen != 0)
                        {
                            status = Status::NotFinished;
                            throw BadStream();
                        }
                        checkEndMarkNow = true;
                    }

                    if (this->needInitState)
                        InitStateReal();

                    if (this->tempBufSize == 0)
                    {
                        const Byte *bufLimit;
                        if (inSize < LZMA_REQUIRED_INPUT_MAX || checkEndMarkNow)
                        {
                            auto dummyRes = TryDummy(srcBytes, inSize);

                            if (dummyRes == DUMMY_ERROR)
                            {
                                memcpy(this->tempBuf, srcBytes, inSize);
                                this->tempBufSize = (unsigned)inSize;
                                srcLen += inSize;
                                status = Status::NeedsMoreInput;
                                return;
                            }

                            if (checkEndMarkNow && dummyRes != DUMMY_MATCH)
                            {
                                status = Status::NotFinished;
                                throw BadStream();
                            }

                            bufLimit = srcBytes;
                        }
                        else
                        {
                            bufLimit = srcBytes + inSize - LZMA_REQUIRED_INPUT_MAX;
                        }

                        this->buf = srcBytes;

                        DecodeReal2(dicLimit, bufLimit);

                        auto processed = std::size_t(this->buf - srcBytes);
                        srcLen += processed;
                        srcBytes += processed;
                        inSize -= processed;
                    }
                    else
                    {
                        unsigned rem = this->tempBufSize, lookAhead = 0;

                        while (rem < LZMA_REQUIRED_INPUT_MAX && lookAhead < inSize)
                            this->tempBuf[rem++] = srcBytes[lookAhead++];

                        this->tempBufSize = rem;

                        if (rem < LZMA_REQUIRED_INPUT_MAX || checkEndMarkNow)
                        {
                            auto dummyRes = TryDummy(this->tempBuf, rem);
                            if (dummyRes == DUMMY_ERROR)
                            {
                                srcLen += lookAhead;
                                status = Status::NeedsMoreInput;
                                return;
                            }
                            if (checkEndMarkNow && dummyRes != DUMMY_MATCH)
                            {
                                status = Status::NotFinished;
                                throw BadStream();
                            }
                        }

                        this->buf = this->tempBuf;

                        DecodeReal2(dicLimit, this->buf);

                        lookAhead -= (rem - (unsigned)(this->buf - this->tempBuf));
                        srcLen += lookAhead;
                        srcBytes += lookAhead;
                        inSize -= lookAhead;
                        this->tempBufSize = 0;
                    }
                }

                if (this->m_code == 0)
                    status = Status::FinishedWithMark;

                if (this->m_code != 0)
                    throw BadStream();
            }

            DictView m_dic;
            Properties m_properties;
            Prob* m_probs;

        private:
            void InitStateReal()
            {
                auto numProbs = Literal + ((UInt32)LZMA_LIT_SIZE << (m_properties.lc + m_properties.lp));

                for (auto i = 0u; i < numProbs; i++)
                    m_probs[i] = kBitModelTotal >> 1;

                this->reps[0] = 1;
                this->reps[1] = 1;
                this->reps[2] = 1;
                this->reps[3] = 1;
                this->state = 0;
                this->needInitState = false;
            }

            void InitRc(const Byte *data)
            {
                m_code = ((UInt32)data[1] << 24) | ((UInt32)data[2] << 16) | ((UInt32)data[3] << 8) | ((UInt32)data[4]);
                m_range = 0xFFFFFFFF;
                needFlush = false;
            }

            void DecodeReal2(std::size_t limit, const Byte *bufLimit)
            {
                do
                {
                    auto limit2 = limit;
                    if (this->checkDicSize == 0)
                    {
                        UInt32 rem = m_properties.dicSize - this->processedPos;
                        if (limit - m_dic.pos > rem)
                            limit2 = m_dic.pos + rem;
                    }

                    DecodeReal(limit2, bufLimit);

                    if (this->processedPos >= m_properties.dicSize)
                        this->checkDicSize = m_properties.dicSize;

                    WriteRem(limit);
                }
                while (m_dic.pos < limit && this->buf < bufLimit && this->remainLen < kMatchSpecLenStart);

                if (this->remainLen > kMatchSpecLenStart)
                    this->remainLen = kMatchSpecLenStart;
            }

            /* First LZMA-symbol is always decoded.
            And it decodes new LZMA-symbols while (buf < bufLimit), but "buf" is without last normalization
            Out:
                Result:
                    SZ_OK - OK
                    SZ_ERROR_DATA - Error
                this->remainLen:
                    < kMatchSpecLenStart : normal remain
                    = kMatchSpecLenStart : finished
                    = kMatchSpecLenStart + 1 : Flush marker
                    = kMatchSpecLenStart + 2 : State Init Marker
            */
            void DecodeReal(std::size_t limit, const Byte *bufLimit)
            {
                auto probs = m_probs;

                unsigned state = this->state;
                UInt32 rep0 = this->reps[0], rep1 = this->reps[1], rep2 = this->reps[2], rep3 = this->reps[3];
                unsigned pbMask = ((unsigned)1 << (m_properties.pb)) - 1;
                unsigned lpMask = ((unsigned)1 << (m_properties.lp)) - 1;
                unsigned lc = m_properties.lc;

                auto dic = m_dic.mem;
                auto dicBufSize = m_dic.size;
                auto dicPos = m_dic.pos;

                UInt32 processedPos = this->processedPos;
                UInt32 checkDicSize = this->checkDicSize;
                unsigned len = 0;

                const Byte *buf = this->buf;
                UInt32 range = this->m_range;
                UInt32 code = this->m_code;

                auto NORMALIZE = [&]
                {
                    if (range < kTopValue)
                    {
                        range <<= 8;
                        code = (code << 8) | (*buf++);
                    }
                };

                do
                {
                    UInt32 bound;
                    unsigned ttt;

                    unsigned posState = processedPos & pbMask;

                    auto isBit0 = [&](Prob* x) -> bool
                    {
                        ttt = *x;
                        NORMALIZE();
                        bound = (range >> kNumBitModelTotalBits) * ttt;
                        return code < bound;
                    };

                    auto UPDATE_0 = [&](Prob* x)
                    {
                        range = bound;
                        *x = (Prob)(ttt + ((kBitModelTotal - ttt) >> kNumMoveBits));
                    };

                    auto UPDATE_1 = [&](Prob* x)
                    {
                        range -= bound;
                        code -= bound;
                        *x = (Prob)(ttt - (ttt >> kNumMoveBits));
                    };

    #define LZMA_DECODER_DETAILS_GET_BIT2_(x, i, A0, A1) if (isBit0(x)) { UPDATE_0(x); i = (i + i); A0; } else { UPDATE_1(x); i = (i + i) + 1; A1; }

                    auto GET_BIT = [&](Prob* x, unsigned& i)
                    {
                        LZMA_DECODER_DETAILS_GET_BIT2_(x, i, ; , ;)
                    };

                    auto TREE_GET_BIT = [&](Prob* probs, unsigned& i) { GET_BIT(probs + i, i); };
                    auto TREE_DECODE = [&](Prob* probs, unsigned limit, unsigned& i)
                    {
                        i = 1;
                        do
                        {
                            TREE_GET_BIT(probs, i);
                        }
                        while (i < limit);
                        i -= limit;
                    };

                    // #define _LZMA_SIZE_OPT
    #ifdef _LZMA_SIZE_OPT
                    auto TREE_6_DECODE = [&](Prob* probs, unsigned& i) { TREE_DECODE(probs, (1 << 6), i); };
    #else
                    auto TREE_6_DECODE = [&](Prob* probs, unsigned& i)
                    {
                        i = 1;
                        TREE_GET_BIT(probs, i);
                        TREE_GET_BIT(probs, i);
                        TREE_GET_BIT(probs, i);
                        TREE_GET_BIT(probs, i);
                        TREE_GET_BIT(probs, i);
                        TREE_GET_BIT(probs, i);
                        i -= 0x40;
                    };
    #endif

                    auto prob = probs + IsMatch + (state << kNumPosBitsMax) + posState;
                    if (isBit0(prob))
                    {
                        unsigned symbol;
                        UPDATE_0(prob);
                        prob = probs + Literal;
                        if (checkDicSize != 0 || processedPos != 0)
                            prob += (LZMA_LIT_SIZE * (((processedPos & lpMask) << lc) +
                            (dic[(dicPos == 0 ? dicBufSize : dicPos) - 1] >> (8 - lc))));

                        if (state < kNumLitStates)
                        {
                            state -= (state < 4) ? state : 3;
                            symbol = 1;
                            do
                            {
                                GET_BIT(prob + symbol, symbol);
                            }
                            while (symbol < 0x100);
                        }
                        else
                        {
                            unsigned matchByte = m_dic.mem[(dicPos - rep0) + ((dicPos < rep0) ? dicBufSize : 0)];
                            unsigned offs = 0x100;
                            state -= (state < 10) ? 3 : 6;
                            symbol = 1;
                            do
                            {
                                unsigned bit;
                                Prob *probLit;
                                matchByte <<= 1;
                                bit = (matchByte & offs);
                                probLit = prob + offs + bit + symbol;
                                LZMA_DECODER_DETAILS_GET_BIT2_(probLit, symbol, offs &= ~bit, offs &= bit)
                            }
                            while (symbol < 0x100);
                        }
                        dic[dicPos++] = (Byte)symbol;
                        processedPos++;
                        continue;
                    }
                    else
                    {
                        UPDATE_1(prob);
                        prob = probs + IsRep + state;
                        if (isBit0(prob))
                        {
                            UPDATE_0(prob);
                            state += kNumStates;
                            prob = probs + LenCoder;
                        }
                        else
                        {
                            UPDATE_1(prob);
                            if (checkDicSize == 0 && processedPos == 0)
                                throw BadStream();

                            prob = probs + IsRepG0 + state;
                            if (isBit0(prob))
                            {
                                UPDATE_0(prob);
                                prob = probs + IsRep0Long + (state << kNumPosBitsMax) + posState;
                                if (isBit0(prob))
                                {
                                    UPDATE_0(prob);
                                    dic[dicPos] = dic[(dicPos - rep0) + ((dicPos < rep0) ? dicBufSize : 0)];
                                    dicPos++;
                                    processedPos++;
                                    state = state < kNumLitStates ? 9 : 11;
                                    continue;
                                }
                                UPDATE_1(prob);
                            }
                            else
                            {
                                UInt32 distance;
                                UPDATE_1(prob);
                                prob = probs + IsRepG1 + state;
                                if (isBit0(prob))
                                {
                                    UPDATE_0(prob);
                                    distance = rep1;
                                }
                                else
                                {
                                    UPDATE_1(prob);
                                    prob = probs + IsRepG2 + state;
                                    if (isBit0(prob))
                                    {
                                        UPDATE_0(prob);
                                        distance = rep2;
                                    }
                                    else
                                    {
                                        UPDATE_1(prob);
                                        distance = rep3;
                                        rep3 = rep2;
                                    }
                                    rep2 = rep1;
                                }
                                rep1 = rep0;
                                rep0 = distance;
                            }
                            state = state < kNumLitStates ? 8 : 11;
                            prob = probs + RepLenCoder;
                        }
                        {
                            unsigned limit, offset;
                            Prob *probLen = prob + LenChoice;
                            if (isBit0(probLen))
                            {
                                UPDATE_0(probLen);
                                probLen = prob + LenLow + (posState << kLenNumLowBits);
                                offset = 0;
                                limit = (1 << kLenNumLowBits);
                            }
                            else
                            {
                                UPDATE_1(probLen);
                                probLen = prob + LenChoice2;
                                if (isBit0(probLen))
                                {
                                    UPDATE_0(probLen);
                                    probLen = prob + LenMid + (posState << kLenNumMidBits);
                                    offset = kLenNumLowSymbols;
                                    limit = (1 << kLenNumMidBits);
                                }
                                else
                                {
                                    UPDATE_1(probLen);
                                    probLen = prob + LenHigh;
                                    offset = kLenNumLowSymbols + kLenNumMidSymbols;
                                    limit = (1 << kLenNumHighBits);
                                }
                            }
                            TREE_DECODE(probLen, limit, len);
                            len += offset;
                        }

                        if (state >= kNumStates)
                        {
                            UInt32 distance;
                            prob = probs + PosSlot +
                                ((len < kNumLenToPosStates ? len : kNumLenToPosStates - 1) << kNumPosSlotBits);
                            TREE_6_DECODE(prob, distance);
                            if (distance >= kStartPosModelIndex)
                            {
                                unsigned posSlot = (unsigned)distance;
                                int numDirectBits = (int)(((distance >> 1) - 1));
                                distance = (2 | (distance & 1));
                                if (posSlot < kEndPosModelIndex)
                                {
                                    distance <<= numDirectBits;
                                    prob = probs + SpecPos + distance - posSlot - 1;
                                    {
                                        UInt32 mask = 1;
                                        unsigned i = 1;
                                        do
                                        {
                                            LZMA_DECODER_DETAILS_GET_BIT2_(prob + i, i, ; , distance |= mask);
                                            mask <<= 1;
                                        }
                                        while (--numDirectBits != 0);
                                    }
                                }
                                else
                                {
                                    numDirectBits -= kNumAlignBits;
                                    do
                                    {
                                        NORMALIZE();
                                        range >>= 1;

                                        {
                                            UInt32 t;
                                            code -= range;
                                            t = (0 - ((UInt32)code >> 31)); /* (UInt32)((Int32)code >> 31) */
                                            distance = (distance << 1) + (t + 1);
                                            code += range & t;
                                        }
                                        /*
                                        distance <<= 1;
                                        if (code >= range)
                                        {
                                        code -= range;
                                        distance |= 1;
                                        }
                                        */
                                    }
                                    while (--numDirectBits != 0);
                                    prob = probs + Align;
                                    distance <<= kNumAlignBits;
                                    {
                                        unsigned i = 1;
                                        LZMA_DECODER_DETAILS_GET_BIT2_(prob + i, i, ; , distance |= 1);
                                        LZMA_DECODER_DETAILS_GET_BIT2_(prob + i, i, ; , distance |= 2);
                                        LZMA_DECODER_DETAILS_GET_BIT2_(prob + i, i, ; , distance |= 4);
                                        LZMA_DECODER_DETAILS_GET_BIT2_(prob + i, i, ; , distance |= 8);
                                    }
                                    if (distance == (UInt32)0xFFFFFFFF)
                                    {
                                        len += kMatchSpecLenStart;
                                        state -= kNumStates;
                                        break;
                                    }
                                }
                            }
                            rep3 = rep2;
                            rep2 = rep1;
                            rep1 = rep0;
                            rep0 = distance + 1;
                        
                            if (checkDicSize == 0)
                            {
                                if (distance >= processedPos)
                                    throw BadStream();
                            }
                            else if (distance >= checkDicSize)
                            {
                                throw BadStream();
                            }

                            state = (state < kNumStates + kNumLitStates) ? kNumLitStates : kNumLitStates + 3;
                        }

                        len += kMatchMinLen;

                        if (limit == dicPos)
                            throw BadStream();

                        {
                            auto rem = limit - dicPos;
                            auto curLen = ((rem < len) ? (unsigned)rem : len);
                            auto pos = (dicPos - rep0) + ((dicPos < rep0) ? dicBufSize : 0);

                            processedPos += curLen;

                            len -= curLen;
                            if (pos + curLen <= dicBufSize)
                            {
                                Byte *dest = dic + dicPos;
                                ptrdiff_t src = (ptrdiff_t)pos - (ptrdiff_t)dicPos;
                                const Byte *lim = dest + curLen;
                                dicPos += curLen;

                                do
                                {
                                    *(dest) = (Byte)*(dest + src);
                                }
                                while (++dest != lim);
                            }
                            else
                            {
                                do
                                {
                                    dic[dicPos++] = dic[pos];
                                    if (++pos == dicBufSize)
                                        pos = 0;
                                }
                                while (--curLen != 0);
                            }
                        }
                    }
                }
                while (dicPos < limit && buf < bufLimit);
                NORMALIZE();
                this->buf = buf;
                this->m_range = range;
                this->m_code = code;
                this->remainLen = len;
                m_dic.pos = dicPos;
                this->processedPos = processedPos;
                this->reps[0] = rep0;
                this->reps[1] = rep1;
                this->reps[2] = rep2;
                this->reps[3] = rep3;
                this->state = state;

    #undef LZMA_DECODER_DETAILS_GET_BIT2_
            }

            void WriteRem(std::size_t limit)
            {
                if (this->remainLen != 0 && this->remainLen < kMatchSpecLenStart)
                {
                    auto dic = m_dic.mem;
                    auto dicPos = m_dic.pos;
                    auto dicBufSize = m_dic.size;

                    unsigned len = this->remainLen;
                    UInt32 rep0 = this->reps[0];
                    if (limit - dicPos < len)
                        len = (unsigned)(limit - dicPos);

                    if (this->checkDicSize == 0 && m_properties.dicSize - this->processedPos <= len)
                        this->checkDicSize = m_properties.dicSize;

                    this->processedPos += len;
                    this->remainLen -= len;
                    while (len-- != 0)
                    {
                        dic[dicPos] = dic[(dicPos - rep0) + ((dicPos < rep0) ? dicBufSize : 0)];
                        dicPos++;
                    }

                    m_dic.pos = dicPos;
                }
            }

            enum Dummy
            {
                DUMMY_ERROR, /* unexpected end of input stream */
                DUMMY_LIT,
                DUMMY_MATCH,
                DUMMY_REP
            };

            Dummy TryDummy(const Byte *buf, std::size_t inSize)
            {
                auto range = this->m_range;
                auto code = this->m_code;
                auto bufLimit = buf + inSize;
                auto probs = m_probs;
                auto state = this->state;
                Dummy res;

                auto NORMALIZE_CHECK = [&]() -> bool
                {
                    if (range < kTopValue)
                    {
                        if (buf >= bufLimit)
                            return false;

                        range <<= 8;
                        code = (code << 8) | (*buf++);
                    }

                    return true;
                };

                {
                    UInt32 bound;
                    unsigned ttt;
                    unsigned posState = (this->processedPos) & ((1 << m_properties.pb) - 1);

                    auto UPDATE_0_CHECK = [&]{ range = bound; };
                    auto UPDATE_1_CHECK = [&]{ range -= bound; code -= bound; };

                    auto bit0Check = [&](Prob* x) -> bool
                    {
                        ttt = *x;
                        bound = (range >> kNumBitModelTotalBits) * ttt;
                        return code < bound;
                    };

    #define LZMA_DECODER_DETAILS_GET_BIT2_CHECK_(x, i, A0, A1) \
        if (bit0Check(x)) { UPDATE_0_CHECK(); i = (i + i); A0; } else { UPDATE_1_CHECK(); i = (i + i) + 1; A1; }

                    auto GET_BIT_CHECK = [&](Prob* x, unsigned& i)
                    {
                        LZMA_DECODER_DETAILS_GET_BIT2_CHECK_(x, i, ; , ;)
                    };

    #define LZMA_DECODER_DETAILS_TREE_DECODE_CHECK_(probs, limit, i) \
                    { i = 1; do { if (!NORMALIZE_CHECK()) return DUMMY_ERROR; GET_BIT_CHECK(probs + i, i); } while (i < limit); i -= limit; }

                    auto prob = probs + IsMatch + (state << kNumPosBitsMax) + posState;
                    if (!NORMALIZE_CHECK()) return DUMMY_ERROR;
                    if (bit0Check(prob))
                    {
                        UPDATE_0_CHECK();

                        /* if (bufLimit - buf >= 7) return DUMMY_LIT; */

                        prob = probs + Literal;
                        if (this->checkDicSize != 0 || this->processedPos != 0)
                            prob += (LZMA_LIT_SIZE *
                            ((((this->processedPos) & ((1 << (m_properties.lp)) - 1)) << m_properties.lc) +
                            (m_dic.mem[(m_dic.pos == 0 ? m_dic.size : m_dic.pos) - 1] >> (8 - m_properties.lc))));

                        if (state < kNumLitStates)
                        {
                            unsigned symbol = 1;
                            do
                            {
                                if (!NORMALIZE_CHECK()) return DUMMY_ERROR;
                                GET_BIT_CHECK(prob + symbol, symbol);
                            }
                            while (symbol < 0x100);
                        }
                        else
                        {
                            unsigned matchByte = m_dic.mem[m_dic.pos - this->reps[0] +
                                ((m_dic.pos < this->reps[0]) ? m_dic.size : 0)];
                            unsigned offs = 0x100;
                            unsigned symbol = 1;
                            do
                            {
                                unsigned bit;
                                Prob *probLit;
                                matchByte <<= 1;
                                bit = (matchByte & offs);
                                probLit = prob + offs + bit + symbol;
                                if (!NORMALIZE_CHECK()) return DUMMY_ERROR;
                                LZMA_DECODER_DETAILS_GET_BIT2_CHECK_(probLit, symbol, offs &= ~bit, offs &= bit)
                            }
                            while (symbol < 0x100);
                        }
                        res = DUMMY_LIT;
                    }
                    else
                    {
                        unsigned len;
                        UPDATE_1_CHECK();

                        prob = probs + IsRep + state;
                        if (!NORMALIZE_CHECK()) return DUMMY_ERROR;
                        if (bit0Check(prob))
                        {
                            UPDATE_0_CHECK();
                            state = 0;
                            prob = probs + LenCoder;
                            res = DUMMY_MATCH;
                        }
                        else
                        {
                            UPDATE_1_CHECK();
                            res = DUMMY_REP;
                            prob = probs + IsRepG0 + state;
                            if (!NORMALIZE_CHECK()) return DUMMY_ERROR;
                            if (bit0Check(prob))
                            {
                                UPDATE_0_CHECK();
                                prob = probs + IsRep0Long + (state << kNumPosBitsMax) + posState;
                                if (!NORMALIZE_CHECK()) return DUMMY_ERROR;
                                if (bit0Check(prob))
                                {
                                    UPDATE_0_CHECK();
                                    if (!NORMALIZE_CHECK())
                                        return DUMMY_ERROR;

                                    return DUMMY_REP;
                                }
                                else
                                {
                                    UPDATE_1_CHECK();
                                }
                            }
                            else
                            {
                                UPDATE_1_CHECK();
                                prob = probs + IsRepG1 + state;
                                if (!NORMALIZE_CHECK()) return DUMMY_ERROR;
                                if (bit0Check(prob))
                                {
                                    UPDATE_0_CHECK();
                                }
                                else
                                {
                                    UPDATE_1_CHECK();
                                    prob = probs + IsRepG2 + state;
                                    if (!NORMALIZE_CHECK()) return DUMMY_ERROR;
                                    if (bit0Check(prob))
                                    {
                                        UPDATE_0_CHECK();
                                    }
                                    else
                                    {
                                        UPDATE_1_CHECK();
                                    }
                                }
                            }
                            state = kNumStates;
                            prob = probs + RepLenCoder;
                        }
                        {
                            unsigned limit, offset;
                            Prob *probLen = prob + LenChoice;
                            if (!NORMALIZE_CHECK()) return DUMMY_ERROR;
                            if (bit0Check(probLen))
                            {
                                UPDATE_0_CHECK();
                                probLen = prob + LenLow + (posState << kLenNumLowBits);
                                offset = 0;
                                limit = 1 << kLenNumLowBits;
                            }
                            else
                            {
                                UPDATE_1_CHECK();
                                probLen = prob + LenChoice2;
                                if (!NORMALIZE_CHECK()) return DUMMY_ERROR;
                                if (bit0Check(probLen))
                                {
                                    UPDATE_0_CHECK();
                                    probLen = prob + LenMid + (posState << kLenNumMidBits);
                                    offset = kLenNumLowSymbols;
                                    limit = 1 << kLenNumMidBits;
                                }
                                else
                                {
                                    UPDATE_1_CHECK();
                                    probLen = prob + LenHigh;
                                    offset = kLenNumLowSymbols + kLenNumMidSymbols;
                                    limit = 1 << kLenNumHighBits;
                                }
                            }
                            LZMA_DECODER_DETAILS_TREE_DECODE_CHECK_(probLen, limit, len);
                            len += offset;
                        }

                        if (state < 4)
                        {
                            unsigned posSlot;
                            prob = probs + PosSlot +
                                ((len < kNumLenToPosStates ? len : kNumLenToPosStates - 1) <<
                                kNumPosSlotBits);
                            LZMA_DECODER_DETAILS_TREE_DECODE_CHECK_(prob, 1 << kNumPosSlotBits, posSlot);
                            if (posSlot >= kStartPosModelIndex)
                            {
                                int numDirectBits = ((posSlot >> 1) - 1);

                                /* if (bufLimit - buf >= 8) return DUMMY_MATCH; */

                                if (posSlot < kEndPosModelIndex)
                                {
                                    prob = probs + SpecPos + ((2 | (posSlot & 1)) << numDirectBits) - posSlot - 1;
                                }
                                else
                                {
                                    numDirectBits -= kNumAlignBits;
                                    do
                                    {
                                        if (!NORMALIZE_CHECK())
                                            return DUMMY_ERROR;

                                        range >>= 1;
                                        code -= range & (((code - range) >> 31) - 1);
                                        /* if (code >= range) code -= range; */
                                    }
                                    while (--numDirectBits != 0);
                                    prob = probs + Align;
                                    numDirectBits = kNumAlignBits;
                                }
                                {
                                    unsigned i = 1;
                                    do
                                    {
                                        if (!NORMALIZE_CHECK()) return DUMMY_ERROR;
                                        GET_BIT_CHECK(prob + i, i);
                                    }
                                    while (--numDirectBits != 0);
                                }
                            }
                        }
                    }
                }

                if (!NORMALIZE_CHECK())
                    return DUMMY_ERROR;

                return res;

    #undef LZMA_DECODER_DETAILS_GET_BIT2_CHECK_
    #undef LZMA_DECODER_DETAILS_TREE_DECODE_CHECK_
            }

            const Byte *buf;

            UInt32 m_range;
            UInt32 m_code;

            UInt32 processedPos;
            UInt32 checkDicSize;
            unsigned state;
            UInt32 reps[4];
            unsigned remainLen;

            bool needFlush;
            bool needInitState;

            unsigned tempBufSize;
            Byte tempBuf[LZMA_REQUIRED_INPUT_MAX];
        };
    }
}

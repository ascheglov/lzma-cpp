// C++ LZMA2 Decoder
// Original code by Igor Pavlov (LZMA SDK 9.20)
// Ported to C++ by Anatoly Scheglov
// Placed in the public domain

#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <stdexcept>

#include "details/LzmaDecoderCore.hpp"

namespace lzma
{
    namespace details
    {
        /*
        00000000  -  EOS
        00000001 U U  -  Uncompressed Reset Dic
        00000010 U U  -  Uncompressed No Reset
        100uuuuu U U P P  -  LZMA no reset
        101uuuuu U U P P  -  LZMA reset state
        110uuuuu U U P P S  -  LZMA reset state + new prop
        111uuuuu U U P P S  -  LZMA reset state + new prop + reset dic

          u, U - Unpack Size
          P - Pack Size
          S - Props
        */

        struct Decoder2Base
        {
            typedef lzma::Byte Byte;

            static const auto CONTROL_LZMA = 1 << 7;
            static const auto CONTROL_COPY_NO_RESET = 2;
            static const auto CONTROL_COPY_RESET_DIC = 1;
            static const auto CONTROL_EOF = 0;

            enum ELzma2State
            {
                LZMA2_STATE_CONTROL,
                LZMA2_STATE_UNPACK0,
                LZMA2_STATE_UNPACK1,
                LZMA2_STATE_PACK0,
                LZMA2_STATE_PACK1,
                LZMA2_STATE_PROP,
                LZMA2_STATE_DATA,
                LZMA2_STATE_DATA_CONT,
                LZMA2_STATE_FINISHED,
                LZMA2_STATE_ERROR
            };

            static bool isThereProp(unsigned mode) { return mode >= 2; }

            static const auto LC_PLUS_LP_MAX = 4;

            static unsigned dicSizeFromProp(unsigned prop)
            {
                return (2ul | (prop & 1)) << (prop / 2 + 11);
            }
        };
    }

    class Decoder2 : private details::Decoder2Base
    {
    public:
        explicit Decoder2(unsigned prop)
        {
            if (prop > 40)
                throw std::invalid_argument("prop");

            decoder.m_properties.lc = LC_PLUS_LP_MAX;
            decoder.m_properties.lp = 0;
            decoder.m_properties.pb = 0;
            decoder.m_properties.dicSize = (prop == 40) ? 0xFFFFFFFF : dicSizeFromProp(prop);

            m_probsArr.reset(new lzma::details::Prob[lzma::details::DecoderCore::calcProbSize(LC_PLUS_LP_MAX)]);
            decoder.m_probs = &m_probsArr[0];

            Reset();
        }

        void Reset()
        {
            state = LZMA2_STATE_CONTROL;
            needInitDic = true;
            needInitState = true;
            needInitProp = true;

            decoder.m_dic.pos = 0;
            decoder.InitDicAndState(true, true);
        }

        /**
            finishMode:
            It has meaning only if the decoding reaches output limit (*destLen or dicLimit).
            LZMA_FINISH_ANY - use smallest number of input bytes
            LZMA_FINISH_END - read EndOfStream marker after decoding

            Returns:
            SZ_OK
            status:
            LZMA_STATUS_FINISHED_WITH_MARK
            LZMA_STATUS_NOT_FINISHED
            LZMA_STATUS_NEEDS_MORE_INPUT
            SZ_ERROR_DATA - Data error
        */
        void DecodeToDic(std::size_t dicLimit, const void* src, std::size_t& srcLen, FinishMode finishMode, Status& status)
        {
            auto srcBytes = static_cast<const Byte*>(src);
            auto inSize = srcLen;
            srcLen = 0;
            status = Status::NotSpecified;

            while (this->state != LZMA2_STATE_FINISHED)
            {
                auto dicPos = this->decoder.m_dic.pos;
                if (this->state == LZMA2_STATE_ERROR)
                    throw BadStream();

                if (dicPos == dicLimit && finishMode == FinishMode::Any)
                {
                    status = Status::NotFinished;
                    return;
                }

                if (this->state != LZMA2_STATE_DATA && this->state != LZMA2_STATE_DATA_CONT)
                {
                    if (srcLen == inSize)
                    {
                        status = Status::NeedsMoreInput;
                        return;
                    }

                    srcLen++;
                    this->state = UpdateState(*srcBytes++);
                    continue;
                }

                {
                    std::size_t destSizeCur = dicLimit - dicPos;
                    std::size_t srcSizeCur = inSize - srcLen;
                    auto curFinishMode = FinishMode::Any;

                    if (this->unpackSize <= destSizeCur)
                    {
                        destSizeCur = this->unpackSize;
                        curFinishMode = FinishMode::End;
                    }

                    if (isUncompressedState())
                    {
                        if (srcLen == inSize)
                        {
                            status = Status::NeedsMoreInput;
                            return;
                        }

                        if (this->state == LZMA2_STATE_DATA)
                        {
                            auto initDic = (this->control == CONTROL_COPY_RESET_DIC);

                            if (initDic)
                            {
                                this->needInitProp = true;
                                this->needInitState = true;
                            }
                            else if (this->needInitDic)
                            {
                                throw BadStream();
                            }

                            this->needInitDic = false;
                            this->decoder.InitDicAndState(initDic, false);
                        }

                        if (srcSizeCur > destSizeCur)
                            srcSizeCur = destSizeCur;

                        if (srcSizeCur == 0)
                            throw BadStream();

                        this->decoder.UpdateWithUncompressed(srcBytes, srcSizeCur);

                        srcBytes += srcSizeCur;
                        srcLen += srcSizeCur;
                        this->unpackSize -= srcSizeCur;
                        this->state = (this->unpackSize == 0) ? LZMA2_STATE_CONTROL : LZMA2_STATE_DATA_CONT;
                    }
                    else
                    {
                        if (this->state == LZMA2_STATE_DATA)
                        {
                            int mode = getLzmaMode();
                            bool initDic = (mode == 3);
                            bool initState = (mode > 0);
                            if ((!initDic && this->needInitDic) || (!initState && this->needInitState))
                                throw BadStream();

                            this->decoder.InitDicAndState(initDic, initState);
                            this->needInitDic = false;
                            this->needInitState = false;
                            this->state = LZMA2_STATE_DATA_CONT;
                        }
                        if (srcSizeCur > this->packSize)
                            srcSizeCur = this->packSize;

                        this->decoder.DecodeToDic(dicPos + destSizeCur, srcBytes, srcSizeCur, curFinishMode, status);

                        srcBytes += srcSizeCur;
                        srcLen += srcSizeCur;
                        this->packSize -= srcSizeCur;

                        auto outSizeProcessed = this->decoder.m_dic.pos - dicPos;
                        this->unpackSize -= outSizeProcessed;

                        if (status == Status::NeedsMoreInput)
                            return;

                        if (srcSizeCur == 0 && outSizeProcessed == 0)
                        {
                            if (status != Status::MaybeFinishedWithoutMark || this->unpackSize != 0 || this->packSize != 0)
                                throw BadStream();

                            this->state = LZMA2_STATE_CONTROL;
                        }

                        if (status == Status::MaybeFinishedWithoutMark)
                            status = Status::NotFinished;
                    }
                }
            }
            status = Status::FinishedWithMark;
        }

        lzma::details::DecoderCore decoder;

    private:
        Decoder2(const Decoder2&); // = delete;
        void operator=(const Decoder2&); // = delete;

        std::unique_ptr<lzma::details::Prob[]> m_probsArr;

        std::size_t packSize;
        std::size_t unpackSize;

        ELzma2State state;

        unsigned control;

        bool needInitDic;
        bool needInitState;
        bool needInitProp;

        bool isUncompressedState() { return (control & CONTROL_LZMA) == 0; }

        unsigned getLzmaMode() { return (control >> 5) & 3; }

        ELzma2State UpdateState(unsigned b)
        {
            switch (this->state)
            {
            case LZMA2_STATE_CONTROL:
                this->control = b;

                if (this->control == 0)
                    return LZMA2_STATE_FINISHED;

                if (isUncompressedState())
                {
                    if ((this->control & 0x7F) > 2)
                        return LZMA2_STATE_ERROR;
                    this->unpackSize = 0;
                }
                else
                {
                    this->unpackSize = unsigned(this->control & 0x1F) << 16;
                }

                return LZMA2_STATE_UNPACK0;

            case LZMA2_STATE_UNPACK0:
                this->unpackSize |= b << 8;
                return LZMA2_STATE_UNPACK1;

            case LZMA2_STATE_UNPACK1:
                this->unpackSize |= b;
                this->unpackSize++;
                return (isUncompressedState()) ? LZMA2_STATE_DATA : LZMA2_STATE_PACK0;

            case LZMA2_STATE_PACK0:
                this->packSize = b << 8;
                return LZMA2_STATE_PACK1;

            case LZMA2_STATE_PACK1:
                this->packSize |= b;
                this->packSize++;
                return isThereProp(getLzmaMode()) ? LZMA2_STATE_PROP: (this->needInitProp ? LZMA2_STATE_ERROR : LZMA2_STATE_DATA);

            case LZMA2_STATE_PROP:
                {
                    int lc, lp;
                    if (b >= (9 * 5 * 5))
                        return LZMA2_STATE_ERROR;
                    lc = b % 9;
                    b /= 9;
                    this->decoder.m_properties.pb = b / 5;
                    lp = b % 5;

                    if (lc + lp > LC_PLUS_LP_MAX)
                        return LZMA2_STATE_ERROR;

                    this->decoder.m_properties.lc = lc;
                    this->decoder.m_properties.lp = lp;
                    this->needInitProp = false;
                    return LZMA2_STATE_DATA;
                }

            default:
                return LZMA2_STATE_ERROR;
            }
        }

    };

    class BufDecoder2 : private Decoder2
    {
    public:
        explicit BufDecoder2(unsigned props) : Decoder2(props)
        {
            m_internalDict.reset(new lzma::Byte[decoder.m_properties.dicSize]);
            decoder.m_dic.mem = m_internalDict.get();
        }

        using Decoder2::Reset;

        void DecodeToBuf(void* dest, std::size_t& destLen, const void* src, std::size_t& srcLen, FinishMode finishMode, Status& status)
        {
            auto destBytes = static_cast<lzma::Byte*>(dest);
            auto srcBytes = static_cast<const lzma::Byte*>(src);
            auto outSize = destLen;
            auto inSize = srcLen;
            srcLen = 0;
            destLen = 0;
            for (;;)
            {
                auto srcSizeCur = inSize;
                
                if (this->decoder.m_dic.pos == this->decoder.m_dic.size)
                    this->decoder.m_dic.pos = 0;

                auto dicPos = this->decoder.m_dic.pos;
                
                std::size_t outSizeCur;
                FinishMode curFinishMode;
                if (outSize > this->decoder.m_dic.size - dicPos)
                {
                    outSizeCur = this->decoder.m_dic.size;
                    curFinishMode = FinishMode::Any;
                }
                else
                {
                    outSizeCur = dicPos + outSize;
                    curFinishMode = finishMode;
                }

                DecodeToDic(outSizeCur, srcBytes, srcSizeCur, curFinishMode, status);
                srcBytes += srcSizeCur;
                inSize -= srcSizeCur;
                srcLen += srcSizeCur;
                outSizeCur = this->decoder.m_dic.pos - dicPos;
                memcpy(destBytes, this->decoder.m_dic.mem + dicPos, outSizeCur);
                destBytes += outSizeCur;
                outSize -= outSizeCur;
                destLen += outSizeCur;

                if (outSizeCur == 0 || outSize == 0)
                    return;
            }
        }
    private:
        BufDecoder2(const BufDecoder2&); // = delete;
        void operator=(const BufDecoder2&); // = delete;

        std::unique_ptr<lzma::Byte[]> m_internalDict;
    };
    /* ---------- One Call Interface ---------- */

    /**
    finishMode:
        It has meaning only if the decoding reaches output limit (*destLen).
            LZMA_FINISH_ANY - use smallest number of input bytes
            LZMA_FINISH_END - read EndOfStream marker after decoding

    status:
        LZMA_STATUS_FINISHED_WITH_MARK
        LZMA_STATUS_NOT_FINISHED
    */
    inline bool Lzma2Decode(void* dest, std::size_t& destLen, const void* src, std::size_t& srcLen, unsigned prop, FinishMode finishMode, Status& status)
    {
        auto destBytes = static_cast<lzma::Byte*>(dest);
        auto outSize = destLen;
        auto inSize = srcLen;

        destLen = 0;
        srcLen = 0;
        
        Decoder2 decoder(prop);
        decoder.decoder.m_dic.mem = destBytes;
        decoder.decoder.m_dic.size = outSize;

        srcLen = inSize;
        decoder.DecodeToDic(outSize, src, srcLen, finishMode, status);
        destLen = decoder.decoder.m_dic.pos;
        
        return status != Status::NeedsMoreInput;;
    }
}

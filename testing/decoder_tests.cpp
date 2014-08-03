// cpp-lzma tests
// belongs to the public domain

#include <lzma-cpp/Lzma2Decoder.hpp>

#include <cassert>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <vector>

#include "test_data_seq.hpp"

struct Tester
{
    static const auto inBufSize = 4096u;
    char inBuf[inBufSize];

    template<typename SeqGen>
    void operator()(std::string testName, SeqGen&& seqGen)
    {
        std::cout << testName << " : ";

        try
        {
            std::ifstream ifs(testName + ".lzma2", std::ios_base::binary);
            if (!ifs)
                throw std::runtime_error("can't open file");

            auto prop = ifs.get();
            lzma::Decoder2 decoder(prop);
            
            std::vector<lzma::Byte> dict(decoder.decoder.m_properties.dicSize);
            decoder.decoder.m_dic.mem = &dict[0];
            decoder.decoder.m_dic.size = dict.size();
            
            auto inLen = 0u;
            auto inPos = 0u;

            lzma::Status status;

            for (;;)
            {
                if (inPos == inLen)
                {
                    ifs.read(inBuf, inBufSize);
                    inPos = 0;
                    inLen = (unsigned)ifs.gcount();
                }

                if (decoder.decoder.m_dic.pos == decoder.decoder.m_dic.size)
                    decoder.decoder.m_dic.pos = 0;

                auto oldPos = decoder.decoder.m_dic.pos;

                std::size_t srcLen = inLen - inPos;
                decoder.DecodeToDic(decoder.decoder.m_dic.size, inBuf + inPos, srcLen, lzma::FinishMode::Any, status);
                
                inPos += srcLen;
                auto outLen = decoder.decoder.m_dic.pos - oldPos;

                seqGen.compare(decoder.decoder.m_dic.mem + oldPos, outLen);

                if (inLen == 0 || outLen == 0)
                    break;
            }

            if (!seqGen.empty())
                throw std::runtime_error("stream is too short");

            if (status == lzma::Status::NeedsMoreInput)
                throw std::runtime_error("incomplete stream");
        }
        catch (std::exception& e)
        {
            std::cout << " FAILED :\n\t" << e.what()  << std::endl;
            return;
        }

        std::cout << "OK" << std::endl;
    }
};

template<std::size_t N>
std::string decode(const char (&src)[N])
{
    char out[1024];
    const auto prop = 0x18;
    lzma::Status status;
    auto outLen = sizeof(out);
    auto encodedLen = N;
    lzma::Lzma2Decode(out, outLen, src, encodedLen, prop, lzma::FinishMode::End, status);
    assert(status == lzma::Status::FinishedWithMark);
    return std::string(out, outLen);
}

void test_Lzma2Decode()
{
    const char encodedEmpty[] = {0};
    assert(decode(encodedEmpty) == "");

    const char encodedStr[] = {1, 0, 7, 't', 'e', 's', 't', '_', 's', 't', 'r', 0};
    assert(decode(encodedStr) == "test_str");
}

int main()
{
    try
    {
        test_Lzma2Decode();

        std::cout << "decoding files..." << std::endl;
        Tester tester;
        run_tests(tester);

        std::cout << "All done.\n" << std::endl;
    }
    catch (std::exception& e)
    {
        std::cout << "Error: " << e.what() << std::endl;
    }
}

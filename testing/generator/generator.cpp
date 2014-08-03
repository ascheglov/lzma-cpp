// cpp-lzma test data generator
// belongs to the public domain

#include "Lzma2Enc.h"

#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <functional>

ISzAlloc alloc =
{
    [](void*, size_t size){ return malloc(size); },
    [](void*, void* mem){ free(mem); }
};

struct InStream : ISeqInStream
{
    template<typename F>
    InStream(F f) : DoRead(f) { Read = ReadImpl; } 

    // if (input(*size) != 0 && output(*size) == 0) means end_of_stream. (output(*size) < input(*size)) is allowed
    static SRes ReadImpl(void *p, void *buf, size_t *size)
    {
        static_cast<InStream*>(p)->DoRead(buf, *size);
        return SZ_OK;
    }

    std::function<void(void* buf, size_t& size)> DoRead;
};

struct OutStream : ISeqOutStream
{
    std::ostream* os;
    OutStream(std::ostream& out) : os(&out) { Write = WriteImpl; } 

    // Returns: result - the number of actually written bytes. (result < size) means error
    static size_t WriteImpl(void *p, const void *buf, size_t size)
    {
        static_cast<OutStream*>(p)->os->write((const char*)buf, size);
        return size;
    }
};

template<typename F>
void lzma2_encode(F f, std::ostream& out, unsigned& properties)
{
    auto enc = Lzma2Enc_Create(&alloc, &alloc);
    if (enc == 0)
        throw std::bad_alloc();

    CLzma2EncProps props;
    Lzma2EncProps_Init(&props);
    auto res = Lzma2Enc_SetProps(enc, &props);
    if (res != SZ_OK)
        throw std::runtime_error("failed to set LZMA encoder properties");

    properties = Lzma2Enc_WriteProperties(enc);

    InStream inStream(f);
    OutStream outStream(out);

    res = Lzma2Enc_Encode(enc, &outStream, &inStream, nullptr);
    if (res != SZ_OK)
        throw std::runtime_error("encode failed");

    Lzma2Enc_Destroy(enc);
}

std::string lzma2_encode(const char* str, size_t available, unsigned& properties)
{
    std::stringstream ss;
    lzma2_encode([&](void* buf, size_t& size)
    {
        if (size > available)
            size = available;

        memcpy(buf, str, size);
        str += size;
        available -= size;
    }
    , ss, properties);
    return ss.str();
}

struct TestGenerator
{
    template<typename SeqGen>
    void operator()(std::string testName, SeqGen&& seqGen)
    {
        std::cout << testName << " : ";

        auto path = testName + ".lzma2";
        std::ofstream ofs(path, std::ios_base::trunc | std::ios_base::binary);
        if (!ofs)
            throw std::runtime_error("failed to rewrite output file");

        ofs.put(0); // reserve space for properties

        unsigned props;
        lzma2_encode(seqGen, ofs, props);

        ofs.seekp(0);
        ofs.put(static_cast<char>(props));
        ofs.close();

        std::cout << "OK\n";
    }
};

#include "../test_data_seq.hpp"

int main()
{
    TestGenerator testGen;
    run_tests(testGen);
}

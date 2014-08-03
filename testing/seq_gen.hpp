// cpp-lzma tests
// belongs to the public domain

#pragma once

#include <exception>

namespace rand_gen
{
    struct LCG
    {
        LCG() : m_LCG_state(~0ULL) {}

        unsigned char operator()()
        {
            m_LCG_state = m_LCG_state * 6364136223846793005LL + 1;
            return (unsigned char)(m_LCG_state >> 32); 
        }

        unsigned long long m_LCG_state;
    };

    template<typename RangeGen>
    struct seq
    {
        LCG lcg;
        RangeGen rangeGen;
        unsigned char last;

        seq(RangeGen rangeGen, unsigned char first) : rangeGen(rangeGen), last(first) {}

        unsigned char operator()()
        {
            unsigned x = lcg();
            unsigned r = rangeGen();
            if (r != 0)
                last = (unsigned char)(last + x % r - r / 2);

            return last;
        }
    };

    template<typename RangeGen>
    inline seq<RangeGen> make(RangeGen rangeGen, unsigned char first)
    {
        return seq<RangeGen>(rangeGen, first);
    }
}

namespace details
{
    template<typename Seq>
    struct make_seq_gen_state
    {
        Seq seq;
        size_t seq_len;

        make_seq_gen_state(Seq s, size_t len) : seq(s), seq_len(len) {}

        void operator()(void* buf, size_t& n)
        {
            if (n > seq_len)
                n = seq_len;

            for (auto s = (unsigned char*)buf, e = s + n; s != e; ++s)
                *s = seq();

            seq_len -= n;
        }

        void compare(const void* buf, size_t n)
        {
            if (n > seq_len)
                throw std::runtime_error("too long sequence");

            for (auto s = (unsigned char*)buf, e = s + n; s != e; ++s)
            {
                if (*s != seq())
                    throw std::runtime_error("mismatch");
            }

            seq_len -= n;
        }

        bool empty() const
        {
            return seq_len == 0;
        }
    };
}

template<typename Seq>
details::make_seq_gen_state<Seq> make_seq(Seq s, size_t len)
{
    return details::make_seq_gen_state<Seq>(s, len);
}

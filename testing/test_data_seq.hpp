// cpp-lzma tests
// belongs to the public domain

#pragma once

#include "seq_gen.hpp"

template<typename F>
inline void run_tests(F&& test)
{
    test("seq_zero_1K", make_seq(rand_gen::make([]{ return 0; }, 0), 1024));
    test("seq_55_1K", make_seq(rand_gen::make([]{ return 0; }, 0x55), 1024));
    test("seq_FF_1K", make_seq(rand_gen::make([]{ return 0; }, 0xFF), 1024));

    test("seq_slow_rand_1K", make_seq(rand_gen::make([]{ return 1; }, 0xAA), 1024));
    test("seq_rand_1K", make_seq(rand_gen::make([]{ return 256; }, 0xAA), 1024));
    
    test("seq_zero_20M", make_seq(rand_gen::make([]{ return 0; }, 0), 20 * 1024 * 1024));
    test("seq_slow_rand_20M", make_seq(rand_gen::make([]{ return 1; }, 0xAA), 20 * 1024 * 1024));
}

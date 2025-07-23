#!/bin/bash
for STD in gnu++98 c++98 gnu++03 c++03 gnu++11 c++11 gnu++14 c++14 gnu++17 c++17 gnu++20 c++20
do
    ./build-all.sh --c++-std=${STD} 3.8 3.9 3.10 3.11
done


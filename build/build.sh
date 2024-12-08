#!/bin/bash


if [ ! -f ./out/Makefile ]; then
    cmake -B ./out
fi

cd ./out
make clean; make
cd -

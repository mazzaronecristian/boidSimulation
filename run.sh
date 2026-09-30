#!/bin/bash

./benchmark sequential 10; 
./benchmark parallel 10 static auto;
./benchmark parallel 10 static 64;
./benchmark parallel 10 static 256;
./benchmark parallel 10 static 1024;
./benchmark parallel 10 dynamic auto;
./benchmark parallel 10 dynamic 64;
./benchmark parallel 10 dynamic 256;
./benchmark parallel 10 dynamic 1024;
./benchmark parallel 10 guided auto;
./benchmark parallel 10 guided 64;
./benchmark parallel 10 guided 256;
./benchmark parallel 10 guided 1024;

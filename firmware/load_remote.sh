#!/bin/bash

make -j4  || exit 1

HEAD_ADDR=rpi
HEAD_USER=ubuntu
scp build/ch.bin $HEAD_USER@$HEAD_ADDR:/tmp
ssh $HEAD_USER@$HEAD_ADDR '. ~/.zshrc; st-flash write /tmp/ch.bin 0x8000000'



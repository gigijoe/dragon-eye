#!/bin/bash

cp /etc/dragon-eye/index.html /tmp
cd /tmp
python -m SimpleHTTPServer
#nohup python -u -m SimpleHTTPServer > SimpleHTTPServer.log 2>&1 &

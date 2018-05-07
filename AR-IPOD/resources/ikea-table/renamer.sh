#!/bin/sh

i=0
for file in ./frame-*[^.pose].txt; do
  mv "$file" "frame-$i.depth.txt"
  i=$((i+1))
done
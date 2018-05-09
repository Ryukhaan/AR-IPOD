#!/bin/sh

i=0
for file in ./*.pose*; do
  mv "$file" "frame-$i.pose.txt"
  i=$((i+1))
done
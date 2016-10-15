#!/usr/bin/env bash

java -jar core/target/granular-media.jar gen static 1000 .05 .06 0 0.01 10e5 20e5

java -jar core/target/granular-media.jar gen dynamic output/static.dat

java -jar core/target/granular-media.jar sim output/static.dat output/dynamic.dat .5 1e-5 0.001
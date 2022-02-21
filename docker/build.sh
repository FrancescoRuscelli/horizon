#!/usr/bin/env bash

IMAGE=horizon

docker build --build-arg CACHE_DATE="$(date)"--rm -t ${IMAGE} .



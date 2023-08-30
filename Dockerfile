FROM nvidia/cuda:12.2.0-base-ubuntu22.04

RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-dev 

ENV PYTHONUNBUFFERED 1

COPY ./requirements.txt /requirements.txt

RUN pip3 install --no-cache-dir -r requirements.txt

RUN apt-get clean

RUN mkdir /src

WORKDIR /src
COPY ./src /src
RUN adduser user
USER user
FROM python:3.7.5-slim
RUN python --version
ENV PYTHONUNBUFFERED 1

WORKDIR /var/app
COPY requirements.txt ./

RUN python -m pip install -r requirements.txt

ADD . .

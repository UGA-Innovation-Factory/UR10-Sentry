FROM python:3.12

LABEL maintainer="Tomas Letelier"

WORKDIR /usr/src/app

COPY requirements.txt ./

RUN pip install --no-cache-dir --upgrade pip \
  && pip install --no-cache-dir -r requirements.txt

COPY Communication/ ./Communication/
COPY Robot/ ./Robot/
COPY UnifiWebsockets/ ./UnifiWebsockets/

COPY BBoxProcessor.py .
COPY URSentry.py .
COPY dockermain.py .

ENV UNIFI_PASSWORD=''

EXPOSE 502

CMD [ "python", "./dockermain.py" ]
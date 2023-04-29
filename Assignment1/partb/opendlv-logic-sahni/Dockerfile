FROM ubuntu:20.04 as builder
RUN apt-get update && \
	apt-get install -y tzdata && \
	apt-get install -y build-essential cmake software-properties-common && \
	add-apt-repository -y ppa:chrberger/libcluon && \
	apt-get update && \
	apt-get install -y libcluon
	
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get install -y tzdata && \
	ln -fs /usr/share/zoneinfo/Europe/Stockholm /etc/localtime && \
	dpkg-reconfigure --frontend noninteractive tzdata

ADD . /opt/sources
WORKDIR /opt/sources
RUN mkdir /tmp/build && \
cmake /opt/sources && \
make && \
cp multiplier /tmp

# Deploy stage
FROM ubuntu:20.04
COPY --from=builder /tmp/multiplier /opt/sources/
WORKDIR /opt/sources
CMD ["./multiplier"]


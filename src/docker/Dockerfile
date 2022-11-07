# Uses python 3.6.9
FROM osrf/ros:melodic-desktop-full

# Set the working directory to /app
WORKDIR /app

COPY ./docker/requirements.txt .

RUN python -m pip install --upgrade pip
RUN pip install -r requirements.txt
RUN apt-get update


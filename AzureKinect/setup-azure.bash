#!/bin/bash
apt update
apt-get install software-properties-common -y
apt install curl -y
curl -sSL https://packages.microsoft.com/keys/microsoft.asc | apt-key add -
apt-add-repository https://packages.microsoft.com/ubuntu/18.04/prod
apt-get update


FROM pytorch/pytorch:1.1.0-cuda10.0-cudnn7.5-runtime

RUN apt-get update && apt-get install -y apt-transport-https && \
    rm -rf /var/lib/apt/lists/*

RUN echo "deb http://developer.download.nvidia.com/compute/cuda/repos/ubuntu1604/x86_64 /" > /etc/apt/sources.list.d/cuda.list
RUN wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1604/x86_64/7fa2af80.pub -O - | apt-key add -

COPY apt-get-requirements.txt /tmp/apt-get-requirements.txt
RUN apt-get update && \
    apt-get install -y $(cat /tmp/apt-get-requirements.txt) && \
    rm -rf /var/lib/apt/lists/*

COPY pip-requirements.txt /tmp/pip-requirements.txt
RUN pip install -r /tmp/pip-requirements.txt

ENV WORKSPACE /home/root/code
RUN mkdir -p $WORKSPACE && \
    git clone https://github.com/facebookresearch/votenet.git $WORKSPACE/votenet && \
    cd $WORKSPACE/votenet/pointnet2 && \
    python setup.py install


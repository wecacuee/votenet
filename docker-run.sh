PATH_TO_DEMO_FILES=$HOME/dat/votenet/demo_files/demo_files/
docker run --rm -it  -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY -e TERM --runtime nvidia --name votenet -v $PATH_TO_DEMO_FILES:/home/root/code/votenet/demo_files votenet

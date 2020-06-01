#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
    DESTDIR_ARG="--root=$DESTDIR"
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/zheng/robot_ws_zheng/src/openni2_camera/openni2_launch"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/zheng/robot_ws_zheng/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/zheng/robot_ws_zheng/install/lib/python2.7/dist-packages:/home/zheng/robot_ws_zheng/build/openni2_launch/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/zheng/robot_ws_zheng/build/openni2_launch" \
    "/usr/bin/python2" \
    "/home/zheng/robot_ws_zheng/src/openni2_camera/openni2_launch/setup.py" \
    build --build-base "/home/zheng/robot_ws_zheng/build/openni2_launch" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/home/zheng/robot_ws_zheng/install" --install-scripts="/home/zheng/robot_ws_zheng/install/bin"

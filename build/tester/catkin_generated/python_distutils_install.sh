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
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/liam/ros/new_catkin/src/tester"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/liam/ros/new_catkin/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/liam/ros/new_catkin/install/lib/python3/dist-packages:/home/liam/ros/new_catkin/build/tester/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/liam/ros/new_catkin/build/tester" \
    "/usr/bin/python3" \
    "/home/liam/ros/new_catkin/src/tester/setup.py" \
     \
    build --build-base "/home/liam/ros/new_catkin/build/tester" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/liam/ros/new_catkin/install" --install-scripts="/home/liam/ros/new_catkin/install/bin"

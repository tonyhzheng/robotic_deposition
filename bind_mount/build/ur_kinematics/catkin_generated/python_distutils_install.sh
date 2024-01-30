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

echo_and_run cd "/home/docker/bind_mount/src/ur_kinematics"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/docker/bind_mount/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/docker/bind_mount/install/lib/python3/dist-packages:/home/docker/bind_mount/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/docker/bind_mount/build" \
    "/usr/bin/python3.7" \
    "/home/docker/bind_mount/src/ur_kinematics/setup.py" \
    egg_info --egg-base /home/docker/bind_mount/build/ur_kinematics \
    build --build-base "/home/docker/bind_mount/build/ur_kinematics" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/docker/bind_mount/install" --install-scripts="/home/docker/bind_mount/install/bin"

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

echo_and_run cd "/home/robot/workspaces/ur5_controllers/src/universal_robot/ur_driver"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/robot/workspaces/ur5_controllers/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/robot/workspaces/ur5_controllers/install/lib/python2.7/dist-packages:/home/robot/workspaces/ur5_controllers/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/robot/workspaces/ur5_controllers/build" \
    "/usr/bin/python2" \
    "/home/robot/workspaces/ur5_controllers/src/universal_robot/ur_driver/setup.py" \
     \
    build --build-base "/home/robot/workspaces/ur5_controllers/build/universal_robot/ur_driver" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/robot/workspaces/ur5_controllers/install" --install-scripts="/home/robot/workspaces/ur5_controllers/install/bin"

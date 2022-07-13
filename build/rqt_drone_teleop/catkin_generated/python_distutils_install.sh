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

echo_and_run cd "/home/csanrod/TFG/2022-tfg-cristian-sanchez/src/drones/rqt_drone_teleop"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/csanrod/TFG/2022-tfg-cristian-sanchez/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/csanrod/TFG/2022-tfg-cristian-sanchez/install/lib/python3/dist-packages:/home/csanrod/TFG/2022-tfg-cristian-sanchez/build/rqt_drone_teleop/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/csanrod/TFG/2022-tfg-cristian-sanchez/build/rqt_drone_teleop" \
    "/usr/bin/python3" \
    "/home/csanrod/TFG/2022-tfg-cristian-sanchez/src/drones/rqt_drone_teleop/setup.py" \
    egg_info --egg-base /home/csanrod/TFG/2022-tfg-cristian-sanchez/build/rqt_drone_teleop \
    build --build-base "/home/csanrod/TFG/2022-tfg-cristian-sanchez/build/rqt_drone_teleop" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/csanrod/TFG/2022-tfg-cristian-sanchez/install" --install-scripts="/home/csanrod/TFG/2022-tfg-cristian-sanchez/install/bin"

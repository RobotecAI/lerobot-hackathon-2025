#!/usr/bin/env sh

case "$SHELL" in
    *bash)
        . ros2_ws/install/setup.bash
        . ./.venv/bin/activate
        echo "Sourced bash install"
        ;;
    *zsh)
        . ros2_ws/install/setup.zsh
        . ./.venv/bin/activate
        echo "Sourced zsh install"
        ;;
    *fish)
        echo "fish is not supported"
        ;;
    *sh)
        . ros2_ws/install/setup.sh
        . ./.venv/bin/activate
        echo "Sourced sh install."
        ;;
    *)
        echo "Unknown shell: $0"
        ;;
esac


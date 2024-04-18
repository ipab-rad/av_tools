#!/bin/bash

# Define the text to append
BASHRC_SNIPPET=$(cat <<'EOF'

# Autocompletion function for uploading Docker images
_upload_docker_image_to_vehicle_completion() {
    local cur=${COMP_WORDS[COMP_CWORD]}
    local imgs=$(docker images --format "{{.Repository}}:{{.Tag}}" | tr '\n' ' ')
    COMPREPLY=($(compgen -W "${imgs}" -- $cur))
}

complete -F _upload_docker_image_to_vehicle_completion upload_docker_image_to_vehicle.sh

EOF
)

# Append the snippet to .bashrc
echo "$BASHRC_SNIPPET" >> ~/.bashrc

echo "Vehicle upload installed successfully."

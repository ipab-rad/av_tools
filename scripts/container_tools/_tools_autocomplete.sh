_record_rosbag_autocomplete() {
    local cur prev opts
    cur="${COMP_WORDS[COMP_CWORD]}"
    prev="${COMP_WORDS[COMP_CWORD-1]}"

    # Directory containing topic lists presets
    topic_list_dir="/opt/ros_ws/config/recording_presets"

    COMPREPLY=()

    # Check if the previous argument was -f or --file
    if [[ "$prev" == "-f" || "$prev" == "--file" ]]; then
        COMPREPLY=( $(compgen -W "$(ls "$topic_list_dir"/*.txt 2>/dev/null)" -- "$cur") )
    fi
}

# Register autocomplete function
complete -F _record_rosbag_autocomplete record_rosbag.sh

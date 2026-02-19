# docker/Dockerfile
#
# Canonical build strategy:
# - Reuse the locally validated base image `iiia:nao`
# - Overlay only repository sources (src/) and rebuild changed packages
# This keeps behavior aligned with the proven runtime stack.
FROM iiia:nao

ENV DEBIAN_FRONTEND=noninteractive

WORKDIR /home/ubuntu/ws

# Overlay repository sources into the runtime workspace.
COPY src/ /home/ubuntu/ws/src/

# Rebuild only the chatbot package on top of the validated base.
RUN bash -c "source /opt/ros/jazzy/setup.bash && \
    cd /home/ubuntu/ws && \
    colcon build --symlink-install --packages-select nao_chatbot nao_posture_bridge"

# Keep entrypoint from this repo for deterministic sourcing behavior.
COPY docker/ros_entrypoint.sh /
RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]

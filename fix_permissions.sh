# Fix ownership of workspace directory
# Docker containers may create files as root, which breaks access for user 'mxck'
# This ensures the directory is always owned by the correct user
sudo chown -R mxck:mxck /home/mxck/mxck2_ws/
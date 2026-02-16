#!/bin/bash

bash /app/src/tasks.sh

# Execute the main command (the CMD from docker-compose or Dockerfile)
exec "$@"
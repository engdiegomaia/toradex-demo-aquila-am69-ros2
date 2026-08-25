# HMI container — static cockpit bundle behind nginx.
#
# Builds for: linux/amd64 AND linux/arm64. nginx:alpine publishes both.
#
# There is deliberately NO build step: no npm install, no bundler, no
# node_modules (plano-cockpit-web.md Decisão 5). The bundle is HTML/CSS/ES
# modules copied verbatim, which is what keeps the arm64 image reproducible and
# keeps the npm supply chain out of the module. If you find yourself adding a
# `RUN npm ...` here, the dependency belongs in the ROS layer or nowhere.
#
# Build (from the repo root, not from docker/):
#   docker build -f docker/hmi/Dockerfile -t local/demo-aquila-hmi:dev .
FROM nginx:alpine

# Restrict envsubst to this project's variables. See the header of
# default.conf.template — without the filter, nginx's own $uri/$host would be
# blanked out and every request would 404 with a valid-looking config.
ENV NGINX_ENVSUBST_FILTER=^COCKPIT_

# Defaults so the image runs standalone (`docker run -p 8081:8081 ...`) without
# compose. Compose overrides all four from .env.
ENV COCKPIT_HMI_PORT=8081 \
    COCKPIT_ROSBRIDGE_PORT=9090 \
    COCKPIT_VIDEO_PORT=8080 \
    COCKPIT_BACKEND_HOST="" \
    COCKPIT_BUILD=dev

COPY docker/hmi/default.conf.template /etc/nginx/templates/default.conf.template

# The stock site would otherwise stay on port 80 alongside ours and answer with
# the nginx welcome page, which is a confusing thing to hit while debugging.
RUN rm -f /etc/nginx/conf.d/default.conf

# Context is the REPO ROOT (see the build command in the header). test/ and
# package.json are host-side developer artifacts — `node --test` and the ESM
# format declaration — and have no business being served.
COPY hmi/ /usr/share/nginx/html/
RUN rm -rf /usr/share/nginx/html/test /usr/share/nginx/html/package.json

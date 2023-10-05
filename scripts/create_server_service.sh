#!/bin/bash

sudo cp murin_server.service /home/nhattm/.config/systemd/user
systemctl --user daemon-reload
systemctl --user enable murin_server.service
systemctl --user start murin_server.service

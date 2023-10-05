#!/bin/bash

sudo cp murin_client.service /home/nhattm/.config/systemd/user
systemctl --user daemon-reload
systemctl --user enable murin_client.service
systemctl --user start murin_client.service

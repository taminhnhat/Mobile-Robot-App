#!/bin/bash

systemctl --user stop murin_server.service
systemctl --user disable murin_server.service
sudo rm /home/nhattm/.config/systemd/user/murin_server.service
systemctl --user daemon-reload

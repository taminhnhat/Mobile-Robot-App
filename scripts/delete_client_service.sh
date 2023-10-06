#!/bin/bash

systemctl --user stop murin_client.service
systemctl --user disable murin_client.service
sudo rm /home/nhattm/.config/systemd/user/murin_client.service
systemctl --user daemon-reload

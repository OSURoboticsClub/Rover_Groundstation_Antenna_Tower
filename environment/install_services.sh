cp *.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable startCanService.service
sudo systemctl enable startPiService.service

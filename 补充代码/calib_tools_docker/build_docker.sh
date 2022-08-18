docker build -t calib-tools:v0 -f docker/cpu.Dockerfile .
sudo apt install x11-xserver-utils -y
docker-compose down && docker-compose up

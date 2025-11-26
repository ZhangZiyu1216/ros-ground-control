# Makefile
# 最终二进制名字，和 installService() 里的一致
BINARY_NAME=ros-ground-control

# 本机本地调试用构建（当前架构）
build:
	@echo "Building for local machine..."
	go build -o bin/$(BINARY_NAME) cmd/server/main.go

# 一键交叉编译：生成 linux/amd64 和 linux/arm64 两个版本
dist:
	@echo "Building for Linux AMD64 (工控机)..."
	CGO_ENABLED=0 GOOS=linux GOARCH=amd64 go build -o dist/linux-amd64/$(BINARY_NAME) cmd/server/main.go
	@echo "Building for Linux ARM64 (Jetson/Pi)..."
	CGO_ENABLED=0 GOOS=linux GOARCH=arm64 go build -o dist/linux-arm64/$(BINARY_NAME) cmd/server/main.go
	@echo "Done. Binaries are in dist/ folder."

clean:
	rm -rf bin dist
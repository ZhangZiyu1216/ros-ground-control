BINARY_NAME=ros-ground-control
# 优化参数: 去除符号表和调试信息
LDFLAGS=-ldflags "-s -w"

build:
	@echo "Building for local machine (Optimized)..."
	go build $(LDFLAGS) -o bin/$(BINARY_NAME) cmd/server/main.go

dist:
	@echo "Building for Linux AMD64 (IPC)..."
	CGO_ENABLED=0 GOOS=linux GOARCH=amd64 go build $(LDFLAGS) -o dist/linux-amd64/$(BINARY_NAME) cmd/server/main.go
	@echo "Building for Linux ARM64 (Jetson/Pi)..."
	CGO_ENABLED=0 GOOS=linux GOARCH=arm64 go build $(LDFLAGS) -o dist/linux-arm64/$(BINARY_NAME) cmd/server/main.go
	@echo "Done. Optimized binaries are in dist/ folder."

clean:
	rm -rf bin dist
# 源文件
cppsrc = $(wildcard ./*.cpp) $(wildcard ../include/*.cpp) $(wildcard ../Coordinate-Transforms/src/*.cpp)
obj = $(cppsrc:.cpp=.o)
dep = $(obj:.o=.d)

# 编译器和标志
CXX = g++
CXXFLAGS = -Wall -std=c++14 -pedantic -fpermissive
CPPFLAGS = -I../include -I../data/kitti

# 链接库标志（移除了错误的 -opf）
LDFLAGS = -lgsl -lgslcblas -larmadillo -lgps

# 目标文件
TARGET = test.exe

# 默认目标
all: $(TARGET)

$(TARGET): $(obj)
	$(CXX) -o $(CFLAGS) $@ $^ $(LDFLAGS)

# 编译规则
%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(CPPFLAGS) -c $< -o $@

# 依赖文件生成
%.d: %.cpp
	@$(CXX) $(CXXFLAGS) $(CPPFLAGS) $< -MM -MT $(@:.d=.o) >$@

-include $(dep)

.PHONY: clean all
clean:
	rm -f $(obj) $(TARGET) $(dep)

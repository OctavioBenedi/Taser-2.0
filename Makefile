# Be carefull. Don't append extension to SRCS (.c or .h)
SRCS := taser

##########################
all: x86

x86:
	@echo "x86 building"
	@echo linking $@
	@echo "astyle --style=allman $(SRCS).c"
	@astyle --style=allman $(SRCS).c
	@echo "gcc $(SRCS).c -Wl,-warn-common -Os $(LDFLAGS) -o $(SRCS)"
	@gcc $(SRCS).c -Wl,-warn-common -Os -o $(SRCS) -lpthread
	@echo "strip --strip-unneeded $(SRCS) $(SRCS)"
	@strip --strip-unneeded $(SRCS)

clean:
	@echo "Cleaning"
	@rm -f $(SRCS).c.orig $(SRCS)

install:
	@echo "Install"
	@sudo cp taser /usr/bin


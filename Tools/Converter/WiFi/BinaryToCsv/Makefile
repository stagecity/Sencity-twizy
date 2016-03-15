CC = gcc
LD = gcc

DEBUGFLAGS = -g -Og

CFLAGS = -Wall -Wextra -Wformat -pedantic -std=gnu11 -O2 -march=native -mtune=native -pipe
LDFLAGS = 
INCLUDE = 

SRCDIR = src
HDIR = headers

RELEASEDIR = release
DEBUGDIR = debug

OBJDIR = obj
TARGETDIR = bin

TARGETNAME = wifiBinToCsv

ifdef HDIR
INCLUDE += -I $(HDIR)
endif


ifndef _OBJDIR

.PHONY: all release debug releaseOpt debugOpt cleanRelease cleanDebug distclean valgrind

default all: release

release: releaseOpt 
debug: debugOpt

valgrind: export _TARGET := valgrind
valgrind: debugOpt

cleanRelease: releaseOpt
cleanDebug: debugOpt
cleanDebug cleanRelease : export _TARGET := clean


releaseOpt: export _OBJDIR := $(OBJDIR)/$(RELEASEDIR)
releaseOpt: export _TARGETDIR := $(TARGETDIR)/$(RELEASEDIR)
releaseOpt: export DEPEND := .dependr

debugOpt: export _OBJDIR := $(OBJDIR)/$(DEBUGDIR)
debugOpt: export _TARGETDIR := $(TARGETDIR)/$(DEBUGDIR)
debugOpt: export DEPEND := .dependd
debugOpt: export EXTRA_CFLAGS := $(DEBUGFLAGS)

releaseOpt debugOpt:
	@$(MAKE) $(_TARGET)


distclean:
	@rm -rvf "$(OBJDIR)" "$(TARGETDIR)"
	@rm -vf .dependr .dependd

else

.PHONY: depend clean valgrind

SRCS = $(wildcard $(SRCDIR)/*.c)
OBJS = $(patsubst $(SRCDIR)%.c,$(_OBJDIR)%.o,$(SRCS))

TARGET = $(_TARGETDIR)/$(TARGETNAME)

$(TARGET): $(DEPEND) $(OBJS) | $(_TARGETDIR)
	$(LD) -o $@ $(OBJS) $(CFLAGS) $(EXTRA_CFLAGS) $(LDFLAGS)

depend: $(DEPEND)

$(DEPEND): $(SRCS)
	@rm -vf $@
	$(CC) $(INCLUDE) -MM $^ | sed '/^.*\.o:/ s/^/$(subst /,\/,$(_OBJDIR))\//'> $@


-include $(DEPEND)

$(_OBJDIR)/%.o: $(SRCDIR)/%.c
	$(CC) -o $@ -c $< $(CFLAGS) $(EXTRA_CFLAGS) $(INCLUDE)

$(OBJS): | $(_OBJDIR)

$(_OBJDIR):
	mkdir -p $(_OBJDIR)

$(_TARGETDIR):
	mkdir -p $(_TARGETDIR)

clean:
	@rm -rvf "$(_OBJDIR)" "$(_TARGETDIR)" "$(DEPEND)" 

valgrind: $(TARGET)
	valgrind --tool=memcheck --track-origins=yes --leak-check=full  "./$(TARGET)" $(ARGS)

endif #_OBJDIR

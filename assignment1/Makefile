# =========================================================
# Makefile per il progetto con moduli multipli (drone, blackboard, ecc.)
# =========================================================

# Compilatore e flag
CC      = gcc
CFLAGS  = -Wall -Wextra
LDFLAGS = -lm

# Cartelle
SRCDIR  = src
OBJDIR  = obj
BINDIR  = exec
LOGDIR  = logs

# File oggetto comuni
OBJS_COMMON = $(OBJDIR)/log.o

# Target principali
TARGETS = main drone obstacle blackboard input target watchdog

# =========================================================
# Regola principale
# =========================================================
all: setup $(TARGETS)

# =========================================================
# Compilazione automatica dei file oggetto
# =========================================================
$(OBJDIR)/%.o: $(SRCDIR)/%.c
	@mkdir -p $(OBJDIR)
	$(CC) $(CFLAGS) -c $< -o $@

# =========================================================
# Regole per gli exec
# =========================================================
main: $(SRCDIR)/main.c $(OBJS_COMMON)
	@mkdir -p $(BINDIR)
	$(CC) $(CFLAGS) $^ -o $(BINDIR)/$@ -lncurses

drone: $(SRCDIR)/drone.c $(OBJS_COMMON)
	@mkdir -p $(BINDIR)
	$(CC) $(CFLAGS) $^ -o $(BINDIR)/$@ $(LDFLAGS)

obstacle: $(SRCDIR)/obstacle.c $(OBJS_COMMON)
	@mkdir -p $(BINDIR)
	$(CC) $(CFLAGS) $^ -o $(BINDIR)/$@ $(LDFLAGS)

blackboard: $(SRCDIR)/blackboard.c $(OBJS_COMMON)
	@mkdir -p $(BINDIR)
	$(CC) $(CFLAGS) $^ -o $(BINDIR)/$@ -lncursesw

target: $(SRCDIR)/target.c $(OBJS_COMMON)
	@mkdir -p $(BINDIR)
	$(CC) $(CFLAGS) $^ -o $(BINDIR)/$@ $(LDFLAGS)

input: $(SRCDIR)/input.c $(OBJS_COMMON)
	@mkdir -p $(BINDIR)
	$(CC) $(CFLAGS) $^ -o $(BINDIR)/$@ -lncurses

watchdog: $(SRCDIR)/watchdog.c $(OBJS_COMMON)
	@mkdir -p $(BINDIR)
	$(CC) $(CFLAGS) $^ -o $(BINDIR)/$@ -lncurses

# =========================================================
# Setup cartelle
# =========================================================
setup:
	@mkdir -p $(OBJDIR) $(BINDIR) $(LOGDIR)
	@touch $(LOGDIR)/system.log
	@touch $(LOGDIR)/watchdog.log

# =========================================================
# Pulizia
# =========================================================
clean:
	rm -rf $(OBJDIR)/*.o

cleanall: clean
	rm -rf $(BINDIR)/* $(LOGDIR)/*.log

# =========================================================
# Esecuzione del programma principale
# =================w========================================
run: all
	./$(BINDIR)/main

# =========================================================
# Fine Makefile
# =========================================================

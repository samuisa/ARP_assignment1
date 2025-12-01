# Compilazione dei file oggetto nella cartella obj/
gcc -c src/log.c -o obj/log.o

# Creazione degli eseguibili nella cartella eseguibili/
gcc src/drone.c obj/log.o -o exec/drone -lncurses -lm &
gcc src/obstacles.c obj/log.o -o exec/obstacles -lncurses -lm &
gcc -Wall -Wextra src/window.c obj/log.o -o exec/window -lncursesw &
gcc src/input.c obj/log.o -o exec/input -lncurses &
gcc src/main.c obj/log.o -o exec/main -lncurses

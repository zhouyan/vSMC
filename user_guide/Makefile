.PHONY : all clean

all :
	cd prg; ./make.sh pf.pdf
	cd tab; ./rng_distribution.pl
	cd tab; ./rng_engine.pl
	ulimit -n 4096
	latexmk -silent -f user_guide.tex >/dev/null

clean:
	cd prg; ./make.sh clean
	rm -rf _minted-user_guide
	latexmk -c user_guide.tex

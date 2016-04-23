.PHONY : all clean

all :
	cd cpp; ./make.sh pf.pdf
	cd tab; ./rng_distribution.pl
	cd tab; ./rng_engine.pl
	ulimit -n 4096
	latexmk -silent -f user_guide.tex >/dev/null

clean:
	cd cpp; ./make.sh clean
	rm -rf _minted-user_guide
	latexmk -c user_guide.tex

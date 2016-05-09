.PHONY : all clean

all :
	cd prg; ./make.sh pf.pdf
	cd tab; ./rng_distribution.pl
	cd tab; ./rng_engine.pl
	latexmk -silent -f manual.tex >/dev/null

clean:
	cd prg; ./make.sh clean
	rm -rf _minted-manual
	latexmk -c manual.tex

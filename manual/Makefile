.PHONY : all clean

all :
	cd prg; ./make.sh pf.pdf
	latexmk -silent -f manual.tex >/dev/null

clean:
	cd prg; ./make.sh clean
	rm -rf _minted-manual
	latexmk -c manual.tex

.PHONY : all clean

all :
	cd cpp; ./make.sh; ./pf_seq; Rscript pf.R; cd ..
	latexmk -silent -f user_guide.tex >/dev/null

clean:
	cd cpp; ./make.sh clean; cd ..
	latexmk -c user_guide.tex

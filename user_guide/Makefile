.PHONY : all clean

all :
	cd cpp; ./make.sh pf_seq; ./pf_seq; Rscript pf.R; cd ..
	ulimit -n 4096
	echo `ulimit -n`
	latexmk -silent -f user_guide.tex >/dev/null

clean:
	cd cpp; ./make.sh clean; cd ..
	latexmk -c user_guide.tex

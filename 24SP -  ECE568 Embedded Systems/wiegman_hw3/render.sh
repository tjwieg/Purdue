pandoc --filter pandoc-crossref --citeproc --number-sections \
    --template acm-pandoc-conf.tex \
    -s 'wiegman_hw3.md' \
    -o 'wiegman_hw3.pdf' 
    #--csl=https://raw.githubusercontent.com/citation-style-language/styles/master/acm-sig-proceedings.csl

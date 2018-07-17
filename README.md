To compile this repository:

    git clone --recursive https://github.com/ahmedwael/robdyn-sferes
    cd robdyn-sferes; sh ./compile_all.sh
    In case of an error related to submodule mapping:
    cd robdyn-sferes;
    git rm --cached sferes2/modules/nn2/;
    git submodule update --init;
    Then copy the nn2 files under robdyn-sferes2/modules/nn2 to robdyn-sferes2/sferes2/modules/nn2
    

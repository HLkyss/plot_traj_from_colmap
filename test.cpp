//此代码用于测试数据的提取效果
#include<iostream>
#include<fstream>
#include<string>
#include<vector>
#include <sstream>

int main( int argc, char** argv )
{
    std::ifstream fin;
//    fin.open("../images.txt",std::ios::in);
    fin.open("/home/hl/project/plot_track-colmap/endofworld/images.txt",std::ios::in);
    std::ofstream fout("/home/hl/project/plot_track-colmap/endofworld/output.txt", std::ios::out); // Open the output file.
    if(!fin.is_open())
    {
        std::cerr<<"cannot open the file";
        return 1;
    }

    char line[50000]={0};   //getline (char* s, streamsize n ),作用是从istream中读取至多n个字符保存在s对应的数组中。即使还没读够n个字符，如果遇到换行符'\n'则读取终止（50000是我试出来的能读取所有数据的一个可行值）
    int lineCount = 0;

//从文件中提取“行”
    while(fin.getline(line,sizeof(line))) {

        //跳过前四行注释
        lineCount++;
        if (lineCount <= 4) {
            continue;
        }

        int num;
        double time, tx, ty, tz, qx, qy, qz, qw;
//        char name[24];
        std::vector<double> rest;
        //从“行”中提取“单词”
        std::stringstream word(line);
//        fin >> time >> qw >> qx >> qy >> qz >> tx >> ty >> tz >> num;     //colmap用sfm，没有时间先后顺序，将image_id当作时间
        word >> time >> qw >> qx >> qy >> qz >> tx >> ty >> tz >> num;     //colmap用sfm，没有时间先后顺序，将image_id当作时间
        //if (time - int(time)<0.01)    //后面舍弃了此终止条件，改用num。所需数据的num即对应camera_id，只有一个相机所以所有camera均为1，判断num是否为1即可判断是否为所需数据行
        if (num == 1)
        {
            std::cout << time << " " << qw << " " << qx << " " << qy << " " << qz << " " << tx << " " << ty << " " << tz<<" "<<num<< std::endl;
            fout << time << " " << qw << " " << qx << " " << qy << " " << qz << " "
                 << tx << " " << ty << " " << tz << " " << num << std::endl;
        }

//        // Check for the end of the file.
//        if (fin.eof()) {
//            break;
//        }
    }
    // Close both input and output files.
    fin.close();
    fout.close();

    return 0;
}

/* 有关fin的参考教程：
 * https://zhuanlan.zhihu.com/p/395782246
 * https://zhuanlan.zhihu.com/p/352961501
 * https://zhuanlan.zhihu.com/p/352961501
 */

/*运行结果（根据结果发现，对比image_id和图片名得知，图片的排列顺序确实正好是时间的先后顺序，所以可以用image_id代表time）
1 0.38066 -0.0276326 0.906643 0.179815 2.42527 -0.362374 2.73882 1 IMG_20221128_184139.jpg
2 0.391721 -0.0292101 0.901557 0.181377 2.44131 -0.33488 2.59628 1 IMG_20221128_184141.jpg
3 0.388121 -0.0337114 0.903227 0.18002 2.49816 -0.287509 2.47265 1 IMG_20221128_184142.jpg
4 0.391281 -0.0211127 0.901987 0.18131 2.52119 -0.356035 2.18623 1 IMG_20221128_184145.jpg
5 0.366796 -0.0225312 0.912865 0.177852 2.52953 -0.283174 1.91151 1 IMG_20221128_184146.jpg
6 0.383752 -0.0261654 0.906004 0.176655 2.51826 -0.243673 1.77405 1 IMG_20221128_184148.jpg
7 0.390497 -0.0294319 0.905904 0.161197 2.55959 -0.196483 1.63121 1 IMG_20221128_184149.jpg
8 0.388915 -0.0226302 0.904174 0.17522 2.51931 -0.218235 1.4024 1 IMG_20221128_184150.jpg
9 0.37452 -0.0213629 0.908453 0.184369 2.46944 -0.19712 1.14473 1 IMG_20221128_184151.jpg
10 0.390132 -0.0263128 0.902882 0.178631 2.51425 -0.144674 0.968372 1 IMG_20221128_184152.jpg
11 0.413655 -0.021893 0.892845 0.176745 2.55003 -0.173148 0.779191 1 IMG_20221128_184153.jpg
12 0.400611 -0.0274329 0.898305 0.178345 2.50317 -0.0628402 0.412924 1 IMG_20221128_184154.jpg
13 0.389756 -0.0265354 0.904342 0.171908 2.47096 -0.0438307 0.17468 1 IMG_20221128_184155.jpg
14 0.405772 -0.0227397 0.895281 0.182495 2.56531 0.00594424 -0.0830839 1 IMG_20221128_184156.jpg
15 0.415808 -0.0296612 0.89212 0.174199 2.63283 0.047914 -0.19954 1 IMG_20221128_184158.jpg
16 0.416977 -0.0344973 0.892822 0.166763 2.68902 0.0638378 -0.405803 1 IMG_20221128_184159.jpg
17 0.416473 -0.0303638 0.891418 0.176074 2.63598 0.111665 -0.644509 1 IMG_20221128_184200.jpg
18 0.412837 -0.0313684 0.89293 0.176798 2.644 0.142599 -0.920112 1 IMG_20221128_184201.jpg
19 0.450898 -0.0405037 0.878457 0.152854 2.88313 0.109452 -1.13174 1 IMG_20221128_184202.jpg
23 0.700473 -0.0364173 0.701388 0.126751 3.74855 -0.0985223 -0.0245325 1 IMG_20221128_184210.jpg
24 0.73609 -0.0322952 0.663838 0.128249 3.86435 -0.128067 0.119371 1 IMG_20221128_184211.jpg
25 0.772676 -0.0438104 0.62522 0.100759 4.00196 -0.193689 0.454768 1 IMG_20221128_184213.jpg
26 0.811968 -0.0539748 0.574839 0.0857605 4.12302 -0.23888 0.909364 1 IMG_20221128_184214.jpg
27 0.825213 -0.0501506 0.555123 0.0913671 4.12995 -0.231126 0.935872 1 IMG_20221128_184214_1.jpg
28 0.841316 -0.0532153 0.532707 0.0746967 4.05972 -0.277259 0.935952 1 IMG_20221128_184215.jpg
29 0.870621 -0.0517878 0.483965 0.0715239 4.0671 -0.294547 1.17742 1 IMG_20221128_184216.jpg
30 0.891886 -0.0589008 0.44431 0.0604843 4.11789 -0.281666 1.37502 1 IMG_20221128_184217.jpg
31 0.903739 -0.0665685 0.419588 0.0526275 4.14004 -0.257788 1.3486 1 IMG_20221128_184218.jpg
32 0.900905 -0.0595451 0.425638 0.0604723 4.13766 -0.216535 0.976778 1 IMG_20221128_184219.jpg
33 0.910307 -0.0582736 0.406134 0.0547773 4.14855 -0.245019 0.926517 1 IMG_20221128_184220.jpg
34 0.932564 -0.0452409 0.355543 0.0432151 4.15053 -0.305906 0.855494 1 IMG_20221128_184222.jpg
35 0.936574 -0.0363757 0.345462 0.0464915 4.07337 -0.26902 0.586767 1 IMG_20221128_184223.jpg
36 0.952557 -0.0339143 0.300198 0.0369583 4.10525 -0.274977 0.71368 1 IMG_20221128_184224.jpg
37 0.974873 -0.0259218 0.220242 0.0210477 4.02987 -0.347568 1.05703 1 IMG_20221128_184225.jpg
38 0.991368 -0.0319181 0.127043 -0.00558808 3.83214 -0.440293 1.56186 1 IMG_20221128_184226.jpg
39 0.998296 -0.028513 0.048169 -0.0164845 3.64183 -0.501725 2.03843 1 IMG_20221128_184227.jpg
40 0.997959 -0.031601 -0.0391271 -0.0393394 3.25196 -0.599054 2.65243 1 IMG_20221128_184230.jpg
41 0.99443 -0.0256289 -0.0824046 -0.0605114 3.02227 -0.765302 2.90413 1 IMG_20221128_184231.jpg
42 0.991977 -0.0171518 -0.0993155 -0.0763083 2.81642 -0.86894 2.7941 1 IMG_20221128_184232.jpg
43 0.971657 -0.0116923 -0.221898 -0.0806686 2.41272 -0.836004 3.31766 1 IMG_20221128_184233.jpg
44 0.964195 -0.0157587 -0.245569 -0.0988726 2.15753 -0.859458 3.21714 1 IMG_20221128_184234.jpg
45 0.947016 -0.0139245 -0.29857 -0.117566 2.04531 -0.922163 3.09771 1 IMG_20221128_184237.jpg
46 0.945887 -0.00620542 -0.308141 -0.101526 2.20394 -0.83466 2.92901 1 IMG_20221128_184239.jpg
47 0.923375 0.0105145 -0.366616 -0.113405 2.08146 -0.907248 3.02478 1 IMG_20221128_184239_1.jpg
48 0.917886 0.0186377 -0.376579 -0.123799 2.12212 -0.975932 2.85345 1 IMG_20221128_184240.jpg
49 0.908543 0.0255965 -0.392398 -0.141135 2.07762 -1.02458 2.67983 1 IMG_20221128_184241.jpg
50 0.895129 0.0262735 -0.418222 -0.152133 2.02616 -1.02758 2.59945 1 IMG_20221128_184242.jpg
51 0.881895 0.029052 -0.445245 -0.15223 2.02302 -0.931122 2.46833 1 IMG_20221128_184243.jpg
52 0.887555 0.0296301 -0.433531 -0.153033 2.06374 -0.848533 2.12362 1 IMG_20221128_184243_1.jpg
53 0.896387 0.026083 -0.413097 -0.158621 2.08661 -0.808793 1.79981 1 IMG_20221128_184244.jpg
54 0.886145 0.0375954 -0.432139 -0.163061 2.11917 -0.773858 1.64494 1 IMG_20221128_184245.jpg
55 0.884416 0.0258817 -0.435985 -0.164488 2.15978 -0.681922 1.39531 1 IMG_20221128_184246.jpg
56 0.892873 0.0332966 -0.415981 -0.169202 2.22816 -0.608625 0.864682 1 IMG_20221128_184248.jpg
57 0.889425 0.0282618 -0.428163 -0.15748 2.25551 -0.503523 0.846291 1 IMG_20221128_184249.jpg
58 0.887193 0.0357392 -0.432436 -0.156878 2.3278 -0.453845 0.679932 1 IMG_20221128_184250.jpg
59 0.901323 0.0337989 -0.401484 -0.159013 2.29283 -0.28044 0.0892634 1 IMG_20221128_184252.jpg
60 0.884321 0.0247522 -0.433971 -0.170392 2.40763 -0.265794 -0.0234526 1 IMG_20221128_184253.jpg
61 0.89305 -0.0183933 -0.423773 -0.150134 2.3059 -0.255655 -0.370365 1 IMG_20221128_184255.jpg
62 0.855627 0.0420512 -0.487319 -0.169274 2.71001 0.060321 -0.614353 1 IMG_20221128_184257.jpg
63 0.834555 0.0201648 -0.522706 -0.172889 2.82414 0.0725278 -0.713174 1 IMG_20221128_184258.jpg
64 0.815497 0.0159269 -0.550386 -0.178286 2.91587 0.0758747 -0.707277 1 IMG_20221128_184258_1.jpg
65 0.787199 -0.00507634 -0.587562 -0.187252 2.97345 -0.041725 -0.555663 1 IMG_20221128_184259.jpg
66 0.731299 -0.0121169 -0.650894 -0.203448 3.14164 -0.182651 -0.162963 1 IMG_20221128_184300.jpg
67 0.677974 -0.0091018 -0.70506 -0.207744 3.24705 -0.231447 0.214911 1 IMG_20221128_184301.jpg
68 0.629265 0.0019957 -0.74902 -0.207343 3.29484 -0.228266 0.565233 1 IMG_20221128_184302.jpg
69 0.594497 -0.000668979 -0.774751 -0.215254 3.28859 -0.288265 0.744288 1 IMG_20221128_184303.jpg
70 0.581194 -0.00816507 -0.783407 -0.220046 3.25025 -0.326688 0.651796 1 IMG_20221128_184304.jpg
71 0.541973 0.000161142 -0.807786 -0.231837 3.34263 -0.362741 0.769497 1 IMG_20221128_184305.jpg
72 -0.485448 0.0102253 0.842923 0.231768 3.43496 -0.454863 1.11582 1 IMG_20221128_184306.jpg
73 -0.454115 0.0200232 0.859989 0.231943 3.53062 -0.498854 1.09232 1 IMG_20221128_184307.jpg
74 -0.439693 0.0124961 0.869174 0.225946 3.62134 -0.398346 0.944557 1 IMG_20221128_184308.jpg
75 -0.38351 0.0145114 0.893312 0.233889 3.70174 -0.485093 1.22786 1 IMG_20221128_184309.jpg
76 -0.317199 0.0147462 0.918837 0.234322 3.77215 -0.561867 1.64229 1 IMG_20221128_184310.jpg
77 -0.27499 0.00450993 0.931229 0.239109 3.74747 -0.567336 1.76433 1 IMG_20221128_184310_1.jpg
78 -0.232454 0.00146089 0.9445 0.232128 3.69539 -0.524927 1.87324 1 IMG_20221128_184311.jpg
79 -0.161226 0.00162429 0.958453 0.235312 3.57375 -0.614305 2.23701 1 IMG_20221128_184312.jpg
80 -0.0878507 -0.00961856 0.966616 0.240507 3.36176 -0.656627 2.66303 1 IMG_20221128_184313.jpg
81 -0.0156265 -0.00705188 0.969331 0.24516 3.11951 -0.803384 3.00105 1 IMG_20221128_184314.jpg
82 0.0176378 -0.0139727 0.967195 0.253038 3.05923 -0.858874 3.05851 1 IMG_20221128_184314_1.jpg
83 0.0540779 -0.000623691 0.966524 0.250814 2.9839 -0.913444 3.02184 1 IMG_20221128_184315.jpg
84 0.108808 -0.00709792 0.963461 0.244651 2.80462 -0.897105 3.22171 1 IMG_20221128_184316.jpg
85 0.196878 -0.0105946 0.950673 0.239472 2.48516 -0.934973 3.58073 1 IMG_20221128_184317.jpg
86 0.233041 -0.00718564 0.94146 0.2435 2.37689 -1.02494 3.64188 1 IMG_20221128_184318.jpg
87 0.250358 0.0120744 0.940822 0.228097 2.33086 -1.01379 3.41422 1 IMG_20221128_184319.jpg
88 0.268104 0.00263125 0.934283 0.235007 2.33645 -0.989266 3.34512 1 IMG_20221128_184319_1.jpg
89 0.338147 -0.00530421 0.913599 0.225754 2.1076 -0.954305 3.53883 1 IMG_20221128_184320.jpg
90 0.358623 -0.00767577 0.908344 0.215038 2.178 -0.846517 3.32834 1 IMG_20221128_184322.jpg

进程已结束,退出代码0

 */
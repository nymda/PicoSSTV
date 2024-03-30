struct rgb {
    unsigned char r;
    unsigned char g;
    unsigned char b;
    
    bool operator == (const rgb& rhs)
    {
        if (r == rhs.r && g == rhs.g && b == rhs.b) {
            return true;
        }
        return false;
    }

    bool operator != (const rgb& rhs)
    {
        return !(*this == rhs);
    }
    
    rgb(unsigned char R, unsigned char G, unsigned char B) {
        r = R;
        g = G;
        b = B;
    }

    rgb() {
        r = 0;
        g = 0;
        b = 0;
    }
};

unsigned char getY(rgb rgb){
    return clampUC(((66 * (rgb.r) + 129 * (rgb.g) + 25 * (rgb.b) + 128) >> 8) + 16);
}

unsigned char getU(rgb rgb){
    return clampUC(((-38 * (rgb.r) - 74 * (rgb.g) + 112 * (rgb.b) + 128) >> 8) + 128);
}

unsigned char getV(rgb rgb){
    return clampUC(((112 * (rgb.r) - 94 * (rgb.g) - 18 * (rgb.b) + 128) >> 8) + 128);
}
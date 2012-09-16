int main ()
{
    int r = -1;

#ifdef HAS_OBJC_MACRO
#if defined(__OBJC__) && __OBJC__
    r = 0;
#endif
#endif

#ifdef HAS_OBJC2_MACRO
#if defined(__OBJC2__) && __OBJC2__
    r = 0;
#endif
#endif

    return r;
}

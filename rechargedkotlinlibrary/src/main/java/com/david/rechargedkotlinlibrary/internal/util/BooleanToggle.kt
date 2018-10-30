package com.david.rechargedkotlinlibrary.internal.util

class BooleanToggle (private var toggle:Boolean = false){
    private var lastState = false
    fun update(state:Boolean):Boolean{
        if(state && !lastState)
            toggle = !toggle
        lastState = state
        return toggled()
    }
    fun toggled() = toggle
}
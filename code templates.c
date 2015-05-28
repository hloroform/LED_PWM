void blink_red()
{
	if(!Red)
	{
		ClearBit(PORTC,0);
		Red = true;
	}
	else
	{
		SetBit(PORTC,0);
		Red = false;
	}
}

void blink_orange()
{
	if(!Orange)
	{
		ClearBit(PORTC,1);
		Orange = true;
	}
	else
	{
		SetBit(PORTC,1);
		Orange = false;
	}
}

void blink_green()
{
	if(!Green)
	{
		ClearBit(PORTC,2);
		Green = true;
	}
	else
	{
		SetBit(PORTC,2);
		Green = false;
	}
}

void blink_blue()
{
	if(!Blue)
	{
		ClearBit(PORTC,3);
		Blue = true;
	}
	else
	{
		SetBit(PORTC,3);
		Blue = false;
	}
}

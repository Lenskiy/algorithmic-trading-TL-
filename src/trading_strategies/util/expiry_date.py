from datetime import timedelta, datetime

import pandas as pd
from pandas_market_calendars import get_calendar
from pandas.tseries.offsets import CustomBusinessDay

asx_calendar = get_calendar('ASX')
nyse_calendar = get_calendar('XNYS')


def next_week_expiry(date: datetime, weekday: str):
    days_until_weekday = (weekday_mapping[weekday.upper()] - date.weekday()) % 7
    if days_until_weekday == 0:
        next_expire_date = date + timedelta(days=7)
    else:
        next_expire_date = date + timedelta(days=days_until_weekday)

    return next_expire_date


def next_month_expiry(date: datetime, weekday: str, ref_week_num=3):
    first_day_next_month = datetime(date.year if date.month != 12 else date.year + 1, (date.month % 12) + 1, 1)
    days_to_add = (weekday_mapping[weekday.upper()] - first_day_next_month.weekday() + 7) % 7 + (ref_week_num - 1) * 7
    next_expire_date = first_day_next_month + timedelta(days=days_to_add)

    return next_expire_date


def next_expiry_date(date: datetime, is_weekly: bool, is_us=True, weekday="FRI") -> datetime:
    """
    :param date: current date.
    :param weekday: day of the week as three-letter string.
    :param is_weekly: weekly expiry if true; monthly expiry if false.
    :param is_us: using the U.S. calendar, else Australian calendar.
    :return: next expiry date `datetime`.

    @author: Yifan Xiao
    """

    calendar = nyse_calendar if is_us else asx_calendar

    if is_weekly:
        next_date = next_week_expiry(date, weekday)
    else:
        next_date = next_month_expiry(date, weekday)

    return closest_expiration_date(next_date, calendar)


def closest_expiration_date(date: datetime, calendar=nyse_calendar):
    if calendar.valid_days(start_date=date, end_date=date).size > 0:
        return date
    else:
        return calendar.valid_days(start_date=date, end_date=date + timedelta(days=10))[0].replace(
            tzinfo=None).to_pydatetime()


def trading_days(start: datetime, end: datetime):
    return nyse_calendar.valid_days(start_date=start, end_date=end).size


def next_nth_trading_day(current: datetime, n: int):
    return nyse_calendar.valid_days(start_date=current, end_date=current + timedelta(days=365 * 3))[n - 1].replace(
        tzinfo=None).to_pydatetime()


weekday_mapping = {
    'MON': 0,
    'TUE': 1,
    'WED': 2,
    'THU': 3,
    'FRI': 4,
    'SAT': 5,
    'SUN': 6
}

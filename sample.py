def add_statistical_measures(
        df,
        macro_and_other_data = None,
        interval = None,
        **params):
    
    '''
    Add statistical measures to the data
    This function adds statistical measures to the data.
    Don't forget to shift the data before using it if you are using
    the high, low, or close prices and assume that the trade is opened at open.

    IMPORTANT NOTE: Add "stat_" prefix to the new columns that you add to the data.
    This is important because we use this prefix to get the statistical measures
    from the data in the simulator, signal generator, alpha strategiy, and research.

    Args:
        df: pd.DataFrame
            The data

        macro_and_other_data: dict

            It consists of information provided from the AllStockPrices class.
            It contains the dataframes of the other stocks, the market index.
            It is usually passed for the daily data, so that we can use the daily data of the other indices

        interval: str
            The interval of the data
    
    Returns:
        df: pd.DataFrame
    '''

    return df
import React, { Component } from 'react';
import { connect as reduxConnect } from 'react-redux';
import PropTypes from 'prop-types';
import Heading from '../components/Heading';
import Header from '../components/Header';
import IconGroup from '../components/IconGroup';
import Icon from '../components/Icon';
import TelemetryView from './TelemetryView';
import ConfigView from './ConfigView';
import GraphView from './GraphView';
import FieldView from './FieldView';
import OpModeView from './OpModeView';
import Tile from '../components/Tile';
import TileGrid from '../components/TileGrid';
import { connect, disconnect } from '../actions/socket';

class Dashboard extends Component {
  componentDidMount() {
    this.props.dispatch(connect(process.env.HOST, process.env.PORT));
  }

  componentWillUnmount() {
    this.props.dispatch(disconnect());
  }

  render() {
    return (
      <div>
        <Header>
          <Heading text="FTC Dashboard" level={1}>
            <IconGroup>
              {
                this.props.isConnected ?
                  <p>{this.props.pingTime}ms&nbsp;&nbsp;&nbsp;&nbsp;</p>
                  : null
              }
              <Icon icon={this.props.isConnected ? 'wifi' : 'no-wifi'} size="large" />
            </IconGroup>
          </Heading>
        </Header>
        <TileGrid>
          <Tile row={1} col={1}>
            <OpModeView />
          </Tile>
          <Tile row="2 / span 2" col={1}>
            <FieldView />
          </Tile>
          <Tile row="1 / span 3" col={2}>
            <GraphView />
          </Tile>=
          <Tile row="1 / span 2" col={3}>
            <ConfigView />
          </Tile>
          <Tile row={3} col={3}>
            <TelemetryView />
          </Tile>
        </TileGrid>
      </div>
    );
  }
}

Dashboard.propTypes = {
  isConnected: PropTypes.bool.isRequired,
  pingTime: PropTypes.number.isRequired,
  dispatch: PropTypes.func.isRequired
};

const mapStateToProps = ({ socket }) => ({
  isConnected: socket.isConnected,
  pingTime: socket.pingTime
});

export default reduxConnect(mapStateToProps)(Dashboard);
